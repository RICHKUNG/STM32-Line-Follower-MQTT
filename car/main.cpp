#include "mbed.h"
#include "bbcar.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

// WiFi Configuration - Update these with your WiFi credentials
#define WIFI_SSID "YOUR_WIFI_SSID"          // set before building
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"  // set before building

// CONTROL PARAMETERS - Adjust these for testing
const int CAR_SPEED = 50;                   // Base car speed (0-100)
const int MIN_CAR_SPEED = 50;               // Minimum car speed
const int U_SPEED = 60;                     // Speed for U-turn maneuver
const int U_TURN_DURATION_MS = 2400;        // Duration for U-turn maneuver (ms)
const int SHARP_SPEED = 50;                 // Speed for sharp turn

const float OBSTACLE_DISTANCE = 15.0;       // Distance threshold for obstacle detection (cm)
const int OBSTACLE_DETECTION_COUNT = 5;     // Number of consecutive readings needed for U-turn
const int SENSOR_READ_INTERVAL_MS = 20;     // Sensor reading interval (ms)
const int CONTROL_PROCESS_INTERVAL_MS = 20; // Control processing interval (ms)
const int MQTT_YIELD_INTERVAL_MS = 100;     // MQTT yield interval (ms)

const int STOP_DELAY_MS = 500;              // Delay before/after maneuvers (ms)
const int RIGHT_TURN_DURATION_MS = 2500;    // Duration for sharp right turn (ms)

const float TURN_MODERATE = 0.5;            // Moderate turn ratio
const float TURN_GENTLE = 0.2;              // Gentle turn ratio
const float TURN_SLIGHT = 0.1;              // Slight turn ratio

const char *MQTT_HOST = "192.168.1.44";     // MQTT broker IP address
const int MQTT_PORT = 1883;                 // MQTT broker port
const char *CAR_DISTANCE_TOPIC = "sensors/car/distance";
const char *CAR_PATTERN_TOPIC = "sensors/car/pattern";
const char *CAR_PING_TOPIC = "sensors/car/ping";
const char *CAR_STATE_TOPIC = "sensors/car/state";
const char *CAR_CONTROL_TOPIC = "control/car/cmd";
const char *MQTT_CLIENT_ID = "BBCar_IOT";

Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmIn servo0_f(D9), servo1_f(D10);
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);
BusInOut qti_pin(D4, D5, D6, D7);
DigitalInOut pin8(D8);

// MQTT and WiFi
WiFiInterface *wifi;
volatile bool closed = false;

// Make client global so it's accessible everywhere
MQTT::Client<MQTTNetwork, Countdown> *global_client = nullptr;

// State control
enum CarState
{
    LINE_FOLLOWING,
    U_TURNING,
    OFF_LINE,
    MANUAL_CONTROL
};
volatile CarState current_state = LINE_FOLLOWING;
volatile bool remote_auto_mode = true;        // Remote control from F769
volatile int manual_direction = 0;            // 0=stop, 1=forward, 2=right, 3=left, 4=backward
volatile bool manual_command_pending = false; // Flag for immediate manual response

// Add enum for pending turn instructions
enum PendingTurn
{
    NO_TURN,
    TURN_RIGHT_SHARP,
    TURN_LEFT_SHARP
};
volatile PendingTurn pending_turn = NO_TURN;

// Distance tracking
float total_distance = 0.0;

// Replace obstacle_count with distance averaging buffer
float distance_buffer[OBSTACLE_DETECTION_COUNT];
int buffer_index = 0;
bool buffer_full = false;

// Global objects for event-driven sensor reading
EventQueue sensor_queue(96 * EVENTS_EVENT_SIZE);
EventQueue control_queue(32 * EVENTS_EVENT_SIZE);
EventQueue print_queue(64 * EVENTS_EVENT_SIZE);
EventQueue mqtt_queue(64 * EVENTS_EVENT_SIZE);
Thread sensor_thread(osPriorityHigh, 4096);
Thread control_thread(osPriorityHigh, 4096);
Thread mqtt_thread(osPriorityHigh, 8192);
Thread print_thread(osPriorityLow, 4096);

// Sensor data (thread-safe with volatile)
volatile int current_pattern = 0;
volatile float current_ping_distance = 0.0;
volatile bool sensor_data_ready = false;

// Global sensor objects
parallax_qti qti1(qti_pin);
parallax_laserping ping1(pin8);

// Print function for EventQueue
void safe_printf(const char *format, ...)
{
    static char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    printf("%s", buffer);
}

void messageArrived(MQTT::MessageData &md)
{
    MQTT::Message &message = md.message;
    char command[100];
    sprintf(command, "%.*s", message.payloadlen, (char *)message.payload);
    print_queue.call(safe_printf, "[IOT] Control command received: %s\r\n", command);

    // Process control commands
    if (strncmp(command, "MODE:AUTO", 9) == 0)
    {
        remote_auto_mode = true;
        current_state = LINE_FOLLOWING;
    }
    else if (strncmp(command, "MODE:MANUAL", 11) == 0)
    {
        remote_auto_mode = false;
        current_state = MANUAL_CONTROL;
        manual_direction = 0;
        manual_command_pending = true;
    }
    else if (strncmp(command, "DIR:FORWARD", 11) == 0)
    {
        manual_direction = 1;
        manual_command_pending = true;
        print_queue.call(safe_printf, "[IOT] Manual direction: FORWARD\r\n");
    }
    else if (strncmp(command, "DIR:LEFT", 8) == 0)
    {
        manual_direction = 2;
        manual_command_pending = true;
        print_queue.call(safe_printf, "[IOT] Manual direction: LEFT\r\n");
    }
    else if (strncmp(command, "DIR:RIGHT", 9) == 0)
    {
        manual_direction = 3;
        manual_command_pending = true;
        print_queue.call(safe_printf, "[IOT] Manual direction: RIGHT\r\n");
    }
    else if (strncmp(command, "DIR:BACKWARD", 12) == 0)
    {
        manual_direction = 4;
        manual_command_pending = true;
        print_queue.call(safe_printf, "[IOT] Manual direction: BACKWARD\r\n");
    }
    else if (strncmp(command, "DIR:STOP", 8) == 0)
    {
        manual_direction = 0;
        manual_command_pending = true;
        print_queue.call(safe_printf, "[IOT] Manual direction: STOP\r\n");
    }
}

// Function to calculate distance using feedback360 angles
void update_distance()
{
    // Calculate Feedback360 measured distance using the same method as 14_3
    float feedback_distance0 = (car.servo0.targetAngle - car.servo0.angle) * 6.5 * 3.14 / 360.0;
    float feedback_distance1 = (car.servo1.targetAngle - car.servo1.angle) * 6.5 * 3.14 / 360.0;
    float feedback_distance = (feedback_distance0 - feedback_distance1) / 2.0;

    // Add the incremental distance to total
    total_distance += fabs(feedback_distance);

    // Reset target angles to current angles for next measurement
    car.servo0.targetAngle = car.servo0.angle;
    car.servo1.targetAngle = car.servo1.angle;
}

// Consolidated publishing data structure
struct PublishData
{
    enum Type
    {
        DISTANCE,
        PATTERN,
        PING,
        STATE
    } type;
    float float_value;
    int int_value;
    char state_info[64];
    CarState car_state;
};

// Single publishing queue and flag
volatile bool publish_pending = false;

// Unified publishing function
void publish_data_event(PublishData *data)
{
    if (!global_client || !global_client->isConnected())
    {
        delete data;
        publish_pending = false;
        return;
    }

    MQTT::Message message;
    char buff[64];
    const char *topic;

    switch (data->type)
    {
    case PublishData::DISTANCE:
        sprintf(buff, "%.2f", data->float_value);
        topic = CAR_DISTANCE_TOPIC;
        break;
    case PublishData::PATTERN:
        sprintf(buff, "%d", data->int_value);
        topic = CAR_PATTERN_TOPIC;
        break;
    case PublishData::PING:
        sprintf(buff, "%.2f", data->float_value);
        topic = CAR_PING_TOPIC;
        break;
    case PublishData::STATE:
    {
        const char *state_names[] = {"LINE_FOLLOWING", "U_TURNING", "OFF_LINE", "MANUAL_CONTROL"};
        if (strlen(data->state_info) > 0)
        {
            sprintf(buff, "%s:%s", state_names[data->car_state], data->state_info);
        }
        else
        {
            sprintf(buff, "%s", state_names[data->car_state]);
        }
        topic = CAR_STATE_TOPIC;
    }
    break;
    }

    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void *)buff;
    message.payloadlen = strlen(buff);

    int rc = global_client->publish(topic, message);
    if (rc != 0)
    {
        print_queue.call(safe_printf, "[IOT] Failed to publish to %s, error: %d\r\n", topic, rc);
    }

    delete data;
    publish_pending = false;
}

// Simplified publish functions
void publish_distance(float distance)
{
    if (!publish_pending)
    {
        publish_pending = true;
        PublishData *data = new PublishData();
        data->type = PublishData::DISTANCE;
        data->float_value = distance;
        mqtt_queue.call(callback(publish_data_event, data));
    }
    print_queue.call(safe_printf, "[IOT] Publishing distance: %.2f cm\r\n", distance);
}

void publish_pattern(int pattern)
{
    if (!publish_pending)
    {
        publish_pending = true;
        PublishData *data = new PublishData();
        data->type = PublishData::PATTERN;
        data->int_value = pattern;
        mqtt_queue.call(callback(publish_data_event, data));
    }
    // print_queue.call(safe_printf, "[IOT] Publishing pattern: %d\r\n", pattern);
}

void publish_ping(float ping_distance)
{
    if (!publish_pending)
    {
        publish_pending = true;
        PublishData *data = new PublishData();
        data->type = PublishData::PING;
        data->float_value = ping_distance;
        mqtt_queue.call(callback(publish_data_event, data));
    }
    // print_queue.call(safe_printf, "[IOT] Publishing ping distance: %.1f cm\r\n", ping_distance);
}

void publish_state(CarState state, const char *additional_info = "")
{
    if (!publish_pending)
    {
        publish_pending = true;
        PublishData *data = new PublishData();
        data->type = PublishData::STATE;
        data->car_state = state;
        if (additional_info && strlen(additional_info) > 0)
        {
            strncpy(data->state_info, additional_info, sizeof(data->state_info) - 1);
            data->state_info[sizeof(data->state_info) - 1] = '\0';
        }
        else
        {
            data->state_info[0] = '\0';
        }
        mqtt_queue.call(callback(publish_data_event, data));
    }
}

// Simplified sensor data publishing
void publish_all_sensor_data()
{
    if (sensor_data_ready)
    {
        // Publish all sensor data in sequence
        publish_distance(total_distance);
        publish_pattern(current_pattern);
        publish_ping(current_ping_distance);

        char binary_str[8];
        sprintf(binary_str, "%d%d%d%d",
                (current_pattern >> 3) & 1, (current_pattern >> 2) & 1,
                (current_pattern >> 1) & 1, current_pattern & 1);
        print_queue.call(safe_printf, "[IOT] Publishing sensor data - Distance: %.2f cm, Ping: %.1f cm, Pattern: %d (binary: 0b%s)\r\n",
                         total_distance, current_ping_distance, current_pattern, binary_str);

        sensor_data_ready = false;
    }
}

// Event-driven sensor reading functions
void read_sensors()
{
    // Use direct objects
    current_pattern = (int)(qti1);
    current_ping_distance = (float)ping1;

    // Update distance calculation
    update_distance();

    sensor_data_ready = true;

    // Fixed: Use proper binary format conversion
    char binary_str[8];
    sprintf(binary_str, "%d%d%d%d",
            (current_pattern >> 3) & 1, (current_pattern >> 2) & 1,
            (current_pattern >> 1) & 1, current_pattern & 1);
    // Uncomment below line for detailed sensor debugging
    print_queue.call(safe_printf, "[IOT] Sensors read - Pattern: %d (binary: 0b%s), Ping: %.1f cm, Distance: %.2f cm\r\n",
                     current_pattern, binary_str, current_ping_distance, total_distance);
}

// Function to add distance to buffer and calculate average
float add_distance_and_get_average(float distance)
{
    if (distance < 1.0) {
        print_queue.call(safe_printf, "[IOT] Invalid distance %.1f cm, ignoring\r\n", distance);
        return 100.0; // Invalid distance
    }
    distance_buffer[buffer_index] = distance;
    buffer_index = (buffer_index + 1) % OBSTACLE_DETECTION_COUNT;
    
    if (!buffer_full && buffer_index == 0) {
        buffer_full = true;
    }
    
    if (!buffer_full) {
        return distance; // Not enough samples yet, return current distance
    }
    
    // Calculate average
    float sum = 0.0;
    for (int i = 0; i < OBSTACLE_DETECTION_COUNT; i++) {
        sum += distance_buffer[i];
    }
    return sum / OBSTACLE_DETECTION_COUNT;
}

void process_car_control()
{
    static CarState previous_state = LINE_FOLLOWING;

    // Detect marker for "turn right sharp at next branch"
    if (current_pattern == 0b0101 && current_state == LINE_FOLLOWING)
    {
        pending_turn = TURN_RIGHT_SHARP;
        print_queue.call(safe_printf, "[IOT] Marker detected: 0101 (set pending RIGHT SHARP turn)\r\n");
    }
    // Detect marker for "turn left sharp at next branch"
    if (current_pattern == 0b1010 && current_state == LINE_FOLLOWING)
    {
        pending_turn = TURN_LEFT_SHARP;
        print_queue.call(safe_printf, "[IOT] Marker detected: 1010 (set pending LEFT SHARP turn)\r\n");
    }

    // Override state based on remote control
    if (!remote_auto_mode)
    {
        current_state = MANUAL_CONTROL;
    }

    // Publish state change
    if (current_state != previous_state)
    {
        publish_state(current_state);
        previous_state = current_state;
    }

    switch (current_state)
    {
    case LINE_FOLLOWING:
        // Check for obstacle using average distance instead of consecutive detection
        {
            float avg_distance = add_distance_and_get_average(current_ping_distance);
            
            if (buffer_full && avg_distance <= OBSTACLE_DISTANCE)
            {
                current_state = U_TURNING;
                print_queue.call(safe_printf, "[IOT] Average distance %.1f cm <= %.1f cm threshold, switching to U-turn\r\n", 
                               avg_distance, OBSTACLE_DISTANCE);
                publish_state(current_state, "OBSTACLE_DETECTED");
                break;
            }
            else if (buffer_full)
            {
                print_queue.call(safe_printf, "[IOT] Average distance: %.1f cm (safe)\r\n", avg_distance);
            }
        }

        // Line following logic using current pattern
        switch (current_pattern)
        {
        // === 急轉（左右輪差速） ===
        case 0b1000:
            // Sharp left turn: stop left wheel, move right wheel
            car.servo0.set_speed(0);
            car.servo1.set_speed(-SHARP_SPEED);
            break;

        case 0b0001:
            // Sharp right turn: stop right wheel, move left wheel
            car.servo0.set_speed(SHARP_SPEED);
            car.servo1.set_speed(0);
            break;

        case 0b1001:
        case 0b1101:
        case 0b1011:
            // Special sharp right patterns: delay 1.5s
            print_queue.call(safe_printf, "[IOT] Pattern 0x%X detected, executing sharp right turn for 1.5s\r\n", current_pattern);
            publish_state(current_state, "SHARP_RIGHT_SPECIAL");
            car.servo0.set_speed(CAR_SPEED);
            car.servo1.set_speed(0);
            ThisThread::sleep_for(chrono::milliseconds(RIGHT_TURN_DURATION_MS));
            car.goStraight(CAR_SPEED);
            break;

        // === 緩轉（使用 turn 函式） ===
        case 0b1100: // gentle right
            car.turn(CAR_SPEED, TURN_GENTLE);
            break;

        case 0b1110: // slight right
            car.turn(CAR_SPEED, TURN_SLIGHT);
            break;

        case 0b0100: // moderate right
            car.turn(CAR_SPEED, TURN_MODERATE);
            break;

        case 0b0010: // moderate left
            car.turn(CAR_SPEED, -TURN_MODERATE);
            break;

        case 0b0011: // gentle left
            car.turn(CAR_SPEED, -TURN_GENTLE);
            break;

        case 0b0111: // slight left
            car.turn(CAR_SPEED, -TURN_SLIGHT);
            break;

        // === 直行 ===
        case 0b0110:
            car.goStraight(CAR_SPEED);
            break;

        // === 停車 ===
        case 0b1111:
            car.stop();
            break;

        // === 線消失 ===
        case 0b0000:
            current_state = OFF_LINE;
            print_queue.call(safe_printf, "[IOT] Line lost, switching to off-line mode\r\n");
            break;

        // === 預設行為 ===
        default:
            car.goStraight(CAR_SPEED);
            print_queue.call(safe_printf, "[IOT] Unknown pattern: 0x%X, going straight\r\n", current_pattern);
            break;
        }
        break;

    case U_TURNING:
        car.stop();
        car.servo0.set_speed(-U_SPEED); // Left wheel backward
        car.servo1.set_speed(-U_SPEED); // Right wheel forward
        ThisThread::sleep_for(chrono::milliseconds(U_TURN_DURATION_MS));
        car.goStraight(CAR_SPEED);
        ThisThread::sleep_for(chrono::milliseconds(STOP_DELAY_MS));
        current_state = LINE_FOLLOWING;
        print_queue.call(safe_printf, "[IOT] U-turn completed, returning to line following\r\n");
        publish_state(current_state, "U_TURN_COMPLETED");
        break;

    case OFF_LINE:
        // Continuously move back until we see something black (non-0000 pattern)
        if (current_pattern == 0b0000)
        {
            print_queue.call(safe_printf, "[IOT] Off-line: moving back to find line (pattern: 0000)\r\n");
            car.goStraight(-MIN_CAR_SPEED); // Keep moving backward at minimum speed
        }
        else
        {
            // Found something black - stop and switch back to line following
            current_state = LINE_FOLLOWING;
            print_queue.call(safe_printf, "[IOT] Black line detected (pattern: %d), returning to line following\r\n", current_pattern);
            publish_state(current_state, "LINE_FOUND");
        }
        break;

    case MANUAL_CONTROL:
        // Execute manual direction commands immediately when received
        if (manual_command_pending)
        {
            switch (manual_direction)
            {
            case 1:
                car.goStraight(CAR_SPEED);
                break; // Forward
            case 2:
                // Sharp left turn - stop left wheel, move right wheel forward
                car.servo0.set_speed(0);
                car.servo1.set_speed(-CAR_SPEED);
                break;
            case 3:
                // Sharp right turn - stop right wheel, move left wheel forward
                car.servo0.set_speed(CAR_SPEED);
                car.servo1.set_speed(0);
                break;
            case 4:
                car.goStraight(-MIN_CAR_SPEED);
                break; // Backward at minimum speed
            default:
                car.stop();
                break; // Stop (stationary by default)
            }
            manual_command_pending = false;
        }
        break;

    default:
        // Default case: return to line following
        current_state = LINE_FOLLOWING;
        print_queue.call(safe_printf, "[IOT] Unknown state, defaulting to LINE_FOLLOWING\r\n");
        publish_state(current_state, "DEFAULT_FALLBACK");
        car.goStraight(CAR_SPEED);
        break;
    }
}

int main()
{
    // Initialize WiFi
    wifi = WiFiInterface::get_default_instance();
    if (!wifi)
    {
        printf("ERROR: No WiFiInterface found.\r\n");
        return -1;
    }

    printf("\nConnecting to %s...\r\n", WIFI_SSID);
    int ret = wifi->connect(WIFI_SSID, WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0)
    {
        printf("\nConnection error: %d\r\n", ret);
        return -1;
    }

    // Print IP address for debugging MQTT connection issues
    SocketAddress sockAddr;
    wifi->get_ip_address(&sockAddr);
    printf("WiFi connected, IP address: %s\r\n", sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None");

    // Initialize MQTT
    NetworkInterface *net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    global_client = &client; // Set global client pointer

    const char *host = MQTT_HOST;
    const int port = MQTT_PORT;
    printf("Connecting to MQTT broker at %s:%d...\r\n", host, port);

    int rc = mqttNetwork.connect(host, port);
    if (rc != 0)
    {
        printf("\nMQTT connection error: %d\r\n", rc);
        printf("Check if the MQTT broker is running and reachable from this device.\r\n");
        return -1;
    }

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char *)MQTT_CLIENT_ID;

    if ((rc = client.connect(data)) != 0)
    {
        printf("Fail to connect MQTT\r\n");
        return -1;
    }

    // Subscribe to control commands from F769
    if (client.subscribe(CAR_CONTROL_TOPIC, MQTT::QOS0, messageArrived) != 0)
    {
        printf("Fail to subscribe to control commands\r\n");
    }
    else
    {
        printf("Subscribed to control topic: %s\r\n", CAR_CONTROL_TOPIC);
        printf("Publishing distance to: %s\r\n", CAR_DISTANCE_TOPIC);
        printf("Publishing pattern to: %s\r\n", CAR_PATTERN_TOPIC);
        printf("Publishing ping to: %s\r\n", CAR_PING_TOPIC);
        printf("Publishing state to: %s\r\n", CAR_STATE_TOPIC);
    }

    // Initialize distance tracking with current servo positions
    total_distance = 0.0;

    // Start event-driven threads
    printf("[IOT] Starting event-driven threads...\r\n");

    // Start print thread
    print_thread.start(callback(&print_queue, &EventQueue::dispatch_forever));

    // Start sensor reading thread
    sensor_thread.start(callback(&sensor_queue, &EventQueue::dispatch_forever));

    // Start control processing thread
    control_thread.start(callback(&control_queue, &EventQueue::dispatch_forever));

    // Start MQTT thread
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));

    // Set up periodic sensor reading
    sensor_queue.call_every(chrono::milliseconds(SENSOR_READ_INTERVAL_MS), callback(&read_sensors));

    // Set up periodic control processing
    control_queue.call_every(chrono::milliseconds(CONTROL_PROCESS_INTERVAL_MS), callback(&process_car_control));

    car.goStraight(CAR_SPEED);

    // Initialize and publish the starting state
    publish_state(current_state, "SYSTEM_STARTUP");

    printf("[IOT] Event-driven system started successfully\r\n");

    // Main thread now only handles MQTT yield
    while (1)
    {
        client.yield(MQTT_YIELD_INTERVAL_MS);
        // Publish all sensor data periodically
        if (sensor_data_ready)
        {
            publish_all_sensor_data();
        }
        ThisThread::sleep_for(chrono::milliseconds(MQTT_YIELD_INTERVAL_MS));
    }
}