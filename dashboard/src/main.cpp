#include "mbed.h"
#include "stm32f7xx.h"
#include "stm32f769i_discovery.h"
#include "hal_stm_lvgl/tft/tft.h"
#include "hal_stm_lvgl/touchpad/touchpad.h"
#include "lvgl/lvgl.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

// CONTROL PARAMETERS
const char* MQTT_HOST = "192.168.1.44";
const char* CAR_CONTROL_TOPIC = "control/car/cmd";
const char* CAR_DISTANCE_TOPIC = "sensors/car/distance";
const char* CAR_PATTERN_TOPIC = "sensors/car/pattern";
const char* CAR_PING_TOPIC = "sensors/car/ping";        // Add ping topic
const char* CAR_STATE_TOPIC = "sensors/car/state";      // Add state topic
const char* CAR_RESTART_TOPIC = "control/car/restart";  // Add restart topic

// TIMING PARAMETERS
const int MQTT_YIELD_INTERVAL_MS = 100;     // MQTT yield interval (ms)
const int MQTT_CONNECTION_CHECK_MS = 10000; // MQTT connection check interval (ms) - 10 seconds
const int LVGL_TICK_INTERVAL_MS = 5;        // LVGL tick interval (ms)
const int UI_UPDATE_CYCLES = 400;           // UI update every N cycles (400 * 5ms = 2 seconds)

DigitalOut led(LED1);

// GLOBAL VARIABLES
EthInterface *ethernet;
volatile float car_distance = 0.0;
volatile float car_ping_distance = 0.0;
volatile int car_pattern = 0;
volatile char car_state[64] = "UNKNOWN";
volatile bool distance_update_pending = false;
volatile bool pattern_update_pending = false;
volatile bool ping_update_pending = false;
volatile bool state_update_pending = false;
volatile bool is_auto_mode = true;
volatile bool mqtt_connected = false;  // Add connection status

// Simple UI Objects
lv_obj_t *distance_label;
lv_obj_t *ping_label;
lv_obj_t *state_label;                                   // Replace pattern_label with state_label
lv_obj_t *pattern_boxes[4];                              // Keep pattern boxes
lv_obj_t *mode_button;
lv_obj_t *mode_label;
lv_obj_t *restart_button;                                // Add restart button

// Thread management - Remove UI thread, keep only MQTT thread
Thread mqtt_thread(osPriorityHigh, 8192);
EventQueue mqtt_queue(32 * EVENTS_EVENT_SIZE);

MQTT::Client<MQTTNetwork, Countdown> *global_client = nullptr;

// UI update function - now called directly in main thread
void update_ui()
{
    if (!distance_label) {
        printf("update_ui: distance_label is NULL!\r\n");
        return;
    }
    
    // Fix distance formatting - use char buffer instead of direct float formatting
    char dist_str[64];
    sprintf(dist_str, "Total Distance: %.1f cm", car_distance);
    lv_label_set_text(distance_label, dist_str);
    
    if (ping_label) {
        char ping_str[64];
        sprintf(ping_str, "LaserPing: %.1f cm", car_ping_distance);
        lv_label_set_text(ping_label, ping_str);
    }
    
    // Update pattern boxes (keep visual display)
    for (int i = 0; i < 4; ++i) {
        if (pattern_boxes[i]) {
            int bit = (car_pattern >> (3 - i)) & 0x1;
            lv_color_t color = bit ? lv_color_black() : lv_color_white();
            lv_obj_set_style_bg_color(pattern_boxes[i], color, 0);
        }
    }
    
    // Update state display instead of pattern label
    if (state_label) {
        char state_str[80];
        sprintf(state_str, "Car State: %s", car_state);
        lv_label_set_text(state_label, state_str);
    }
    
    // Update mode display with connection status
    if (mode_label) {
        char mode_text[64];
        sprintf(mode_text, "Mode: %s %s", 
                is_auto_mode ? "AUTO" : "MANUAL",
                mqtt_connected ? "[ON]" : "[OFF]");  // Use ASCII instead of Unicode
        lv_label_set_text(mode_label, mode_text);
    }
    if (mode_button) {
        lv_obj_t *btn_label = lv_obj_get_child(mode_button, 0);
        if (btn_label) {
            lv_label_set_text(btn_label, is_auto_mode ? "MANUAL" : "AUTO");
        }
    }
}

// MQTT callbacks - now only set flags, no direct UI updates
void distance_messageArrived(MQTT::MessageData &md)
{
    MQTT::Message &message = md.message;
    char payload[32];
    int len = (message.payloadlen < 31) ? message.payloadlen : 31;
    sprintf(payload, "%.*s", len, (char *)message.payload);
    payload[len] = '\0';
    
    float distance;
    if (sscanf(payload, "%f", &distance) == 1) {
        car_distance = distance;
        distance_update_pending = true;
    } else {
        printf("[F769] Failed to parse distance value: '%s'\r\n", payload);
    }
}

void ping_messageArrived(MQTT::MessageData &md)
{
    MQTT::Message &message = md.message;
    char payload[32];
    int len = (message.payloadlen < 31) ? message.payloadlen : 31;
    sprintf(payload, "%.*s", len, (char *)message.payload);
    payload[len] = '\0';
    
    float ping_distance;
    if (sscanf(payload, "%f", &ping_distance) == 1) {
        car_ping_distance = ping_distance;
        ping_update_pending = true;
    } else {
        printf("[F769] Failed to parse ping value: '%s'\r\n", payload);
    }
}

void pattern_messageArrived(MQTT::MessageData &md)
{
    MQTT::Message &message = md.message;
    char payload[16];
    int len = (message.payloadlen < 15) ? message.payloadlen : 15;
    sprintf(payload, "%.*s", len, (char *)message.payload);
    payload[len] = '\0';
    
    int pattern;
    if (sscanf(payload, "%d", &pattern) == 1) {
        car_pattern = pattern;
        pattern_update_pending = true;
    } else {
        printf("[F769] Failed to parse pattern value: '%s'\r\n", payload);
    }
}

void state_messageArrived(MQTT::MessageData &md)
{
    MQTT::Message &message = md.message;
    int len = (message.payloadlen < 63) ? message.payloadlen : 63;
    sprintf((char*)car_state, "%.*s", len, (char *)message.payload);
    car_state[len] = '\0';
    state_update_pending = true;
}

// MQTT yield function for EventQueue
void mqtt_yield_task()
{
    if (global_client && global_client->isConnected()) {
        mqtt_connected = true;
        int rc = global_client->yield(100);
        if (rc != 0) {
            printf("MQTT yield failed: %d\r\n", rc);
            mqtt_connected = false;
        }
    } else {
        mqtt_connected = false;
    }
}

// MQTT connection check function
void mqtt_connection_check()
{
    if (global_client) {
        bool was_connected = mqtt_connected;
        mqtt_connected = global_client->isConnected();
        
        if (was_connected != mqtt_connected) {
            printf("MQTT connection status changed: %s\r\n", mqtt_connected ? "CONNECTED" : "DISCONNECTED");
        } else {
            printf("MQTT connection check: %s\r\n", mqtt_connected ? "CONNECTED" : "DISCONNECTED");
        }
    } else {
        mqtt_connected = false;
        printf("MQTT connection check: CLIENT NOT INITIALIZED\r\n");
    }
}

void publish_control_command(const char* command) {
    if (global_client == nullptr || !global_client->isConnected()) {
        printf("MQTT client not connected\r\n");
        mqtt_connected = false;
        return;
    }
    
    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) command;
    message.payloadlen = strlen(command);
    
    int rc = global_client->publish(CAR_CONTROL_TOPIC, message);
    if (rc != 0) {
        printf("Publish failed: %d\r\n", rc);
        mqtt_connected = false;
    } else {
        printf("Sent: %s\r\n", command);
        mqtt_connected = true;
    }
}

void forward_press_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        printf("Forward button pressed\r\n");
        publish_control_command("DIR:FORWARD");
    } else if (code == LV_EVENT_RELEASED) {
        printf("Forward button released\r\n");
        publish_control_command("DIR:STOP");
    }
}

void left_press_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        printf("Left button pressed\r\n");
        publish_control_command("DIR:LEFT");
    } else if (code == LV_EVENT_RELEASED) {
        printf("Left button released\r\n");
        publish_control_command("DIR:STOP");
    }
}

void right_press_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        printf("Right button pressed\r\n");
        publish_control_command("DIR:RIGHT");
    } else if (code == LV_EVENT_RELEASED) {
        printf("Right button released\r\n");
        publish_control_command("DIR:STOP");
    }
}

void backward_press_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        printf("Backward button pressed\r\n");
        publish_control_command("DIR:BACKWARD");
    } else if (code == LV_EVENT_RELEASED) {
        printf("Backward button released\r\n");
        publish_control_command("DIR:STOP");
    }
}

void stop_press_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        printf("Stop button pressed\r\n");
        publish_control_command("DIR:STOP");
    }
}

void mode_toggle_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        is_auto_mode = !is_auto_mode;
        // printf("Mode toggle clicked - switching to %s\r\n", is_auto_mode ? "AUTO" : "MANUAL");
        
        // Send mode command to IOT
        if (is_auto_mode) {
            publish_control_command("MODE:AUTO");
        } else {
            publish_control_command("MODE:MANUAL");
        }
        
        // Update UI immediately
        if (mode_label) {
            lv_label_set_text(mode_label, is_auto_mode ? "Mode: AUTO" : "Mode: MANUAL");
        }
        if (mode_button) {
            lv_obj_t *btn_label = lv_obj_get_child(mode_button, 0);
            if (btn_label) {
                lv_label_set_text(btn_label, is_auto_mode ? "MANUAL" : "AUTO");
            }
        }
    }
}

void restart_press_event(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        printf("Restart button pressed - sending sensor reset command\r\n");
        
        // Send restart command via control topic (works in both modes)
        publish_control_command("SYS:RESTART");
        
        // Also reset local sensor displays
        car_distance = 0.0;
        car_ping_distance = 0.0;
        car_pattern = 0;
        strcpy((char*)car_state, "SYSTEM_RESET");
        
        // Force UI update
        distance_update_pending = true;
        ping_update_pending = true;
        pattern_update_pending = true;
        state_update_pending = true;
        
        printf("Sensor reset command sent and local display reset\r\n");
    }
}

void create_simple_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (!scr) {
        printf("LVGL: No active screen!\r\n");
        while (1);
    }

    // Mode status label (top left) - bigger font
    mode_label = lv_label_create(scr);
    if (!mode_label) {
        printf("LVGL: Failed to create mode_label!\r\n");
        while (1);
    }
    lv_label_set_text(mode_label, "Mode: AUTO");
    lv_obj_set_style_text_font(mode_label, &lv_font_montserrat_18, 0);
    lv_obj_align(mode_label, LV_ALIGN_TOP_LEFT, 15, 15);

    // Mode toggle button (top right) - bigger
    mode_button = lv_button_create(scr);
    if (!mode_button) {
        printf("LVGL: Failed to create mode_button!\r\n");
        while (1);
    }
    lv_obj_set_size(mode_button, 120, 50);
    lv_obj_align(mode_button, LV_ALIGN_TOP_RIGHT, -15, 15);
    lv_obj_add_event_cb(mode_button, mode_toggle_event, LV_EVENT_CLICKED, NULL);
    
    // Create button label properly
    lv_obj_t *mode_btn_label = lv_label_create(mode_button);
    if (!mode_btn_label) {
        printf("LVGL: Failed to create mode_btn_label!\r\n");
        while (1);
    }
    lv_label_set_text(mode_btn_label, "MANUAL");
    lv_obj_set_style_text_font(mode_btn_label, &lv_font_montserrat_16, 0);
    lv_obj_center(mode_btn_label);

    // Distance label (top center) - bigger font and better positioning
    distance_label = lv_label_create(scr);
    if (!distance_label) {
        printf("LVGL: Failed to create distance_label!\r\n");
        while (1);
    }
    lv_label_set_text(distance_label, "Total Distance: 0.0 cm");
    lv_obj_set_style_text_font(distance_label, &lv_font_montserrat_16, 0);
    lv_obj_align(distance_label, LV_ALIGN_TOP_MID, 0, 80);

    // LaserPing label (below distance) - bigger font
    ping_label = lv_label_create(scr);
    if (!ping_label) {
        printf("LVGL: Failed to create ping_label!\r\n");
        while (1);
    }
    lv_label_set_text(ping_label, "LaserPing: 0.0 cm");
    lv_obj_set_style_text_font(ping_label, &lv_font_montserrat_16, 0);
    lv_obj_align(ping_label, LV_ALIGN_TOP_MID, 0, 110);

    // Pattern display: 4 rectangles horizontally - keep boxes, no label
    int box_size = 25;  // Smaller boxes since no label
    int spacing = 8;
    int start_x = -((box_size * 4 + spacing * 3) / 2) + box_size/2;
    for (int i = 0; i < 4; ++i) {
        pattern_boxes[i] = lv_obj_create(scr);
        lv_obj_set_size(pattern_boxes[i], box_size, box_size);
        lv_obj_align(pattern_boxes[i], LV_ALIGN_TOP_MID, start_x + i * (box_size + spacing), 140);
        lv_obj_set_style_bg_color(pattern_boxes[i], lv_color_white(), 0);
        lv_obj_set_style_border_width(pattern_boxes[i], 2, 0);
        lv_obj_set_style_border_color(pattern_boxes[i], lv_color_black(), 0);
    }

    // State label (below pattern boxes) - bigger font
    state_label = lv_label_create(scr);
    if (!state_label) {
        printf("LVGL: Failed to create state_label!\r\n");
        while (1);
    }
    lv_label_set_text(state_label, "Car State: UNKNOWN");
    lv_obj_set_style_text_font(state_label, &lv_font_montserrat_16, 0);
    lv_obj_align(state_label, LV_ALIGN_TOP_MID, 0, 175);

    // Restart button (below state label) - Update button text and style
    restart_button = lv_button_create(scr);
    if (!restart_button) {
        printf("LVGL: Failed to create restart_button!\r\n");
        while (1);
    }
    lv_obj_set_size(restart_button, 140, 40);  // Make button wider
    lv_obj_align(restart_button, LV_ALIGN_TOP_MID, 0, 205);
    lv_obj_set_style_bg_color(restart_button, lv_color_make(255, 165, 0), 0);  // Orange color
    lv_obj_add_event_cb(restart_button, restart_press_event, LV_EVENT_CLICKED, NULL);
    
    // Create restart button label
    lv_obj_t *restart_btn_label = lv_label_create(restart_button);
    if (!restart_btn_label) {
        printf("LVGL: Failed to create restart_btn_label!\r\n");
        while (1);
    }
    lv_label_set_text(restart_btn_label, "RESET SENSORS");  // More descriptive text
    lv_obj_set_style_text_font(restart_btn_label, &lv_font_montserrat_14, 0);  // Use available font size
    lv_obj_set_style_text_color(restart_btn_label, lv_color_white(), 0);
    lv_obj_center(restart_btn_label);

    // Direction control buttons - adjust position
    int btn_w = 90, btn_h = 50;
    int btn_spacing = 100;
    int btn_center_y = 130;  // Adjust for restart button

    // Forward button
    lv_obj_t *up_btn = lv_button_create(scr);
    lv_obj_set_size(up_btn, btn_w, btn_h);
    lv_obj_align(up_btn, LV_ALIGN_CENTER, 0, btn_center_y - btn_spacing);
    lv_obj_add_event_cb(up_btn, forward_press_event, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(up_btn, forward_press_event, LV_EVENT_RELEASED, NULL);
    lv_obj_t *up_label = lv_label_create(up_btn);
    lv_label_set_text(up_label, "FWD");
    lv_obj_set_style_text_font(up_label, &lv_font_montserrat_14, 0);
    lv_obj_center(up_label);

    // Left button
    lv_obj_t *left_btn = lv_button_create(scr);
    lv_obj_set_size(left_btn, btn_w, btn_h);
    lv_obj_align(left_btn, LV_ALIGN_CENTER, -btn_spacing, btn_center_y);
    lv_obj_add_event_cb(left_btn, left_press_event, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(left_btn, left_press_event, LV_EVENT_RELEASED, NULL);
    lv_obj_t *left_label = lv_label_create(left_btn);
    lv_label_set_text(left_label, "LEFT");
    lv_obj_set_style_text_font(left_label, &lv_font_montserrat_14, 0);
    lv_obj_center(left_label);

    // Right button
    lv_obj_t *right_btn = lv_button_create(scr);
    lv_obj_set_size(right_btn, btn_w, btn_h);
    lv_obj_align(right_btn, LV_ALIGN_CENTER, btn_spacing, btn_center_y);
    lv_obj_add_event_cb(right_btn, right_press_event, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(right_btn, right_press_event, LV_EVENT_RELEASED, NULL);
    lv_obj_t *right_label = lv_label_create(right_btn);
    lv_label_set_text(right_label, "RIGHT");
    lv_obj_set_style_text_font(right_label, &lv_font_montserrat_14, 0);
    lv_obj_center(right_label);

    // Backward button
    lv_obj_t *back_btn = lv_button_create(scr);
    lv_obj_set_size(back_btn, btn_w, btn_h);
    lv_obj_align(back_btn, LV_ALIGN_CENTER, 0, btn_center_y + btn_spacing);
    lv_obj_add_event_cb(back_btn, backward_press_event, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(back_btn, backward_press_event, LV_EVENT_RELEASED, NULL);
    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_14, 0);
    lv_obj_center(back_label);

    // Stop button (center)
    lv_obj_t *stop_btn = lv_button_create(scr);
    lv_obj_set_size(stop_btn, 70, 70);
    lv_obj_align(stop_btn, LV_ALIGN_CENTER, 0, btn_center_y);
    lv_obj_set_style_bg_color(stop_btn, lv_color_make(255, 0, 0), 0);
    lv_obj_set_style_bg_color(stop_btn, lv_color_make(200, 0, 0), LV_STATE_PRESSED);
    lv_obj_add_event_cb(stop_btn, stop_press_event, LV_EVENT_PRESSED, NULL);
    lv_obj_t *stop_label = lv_label_create(stop_btn);
    lv_label_set_text(stop_label, "STOP");
    lv_obj_set_style_text_color(stop_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(stop_label, &lv_font_montserrat_16, 0);
    lv_obj_center(stop_label);
}

void mqtt_yield_thread(MQTT::Client<MQTTNetwork, Countdown> *client)
{
    printf("MQTT yield thread started\r\n");
    while (1)
    {
        if (client && client->isConnected()) {
            int rc = client->yield(100);
            if (rc != 0) {
                printf("MQTT yield failed: %d\r\n", rc);
            }
        }
        ThisThread::sleep_for(100ms);  // Increased sleep time
    }
}

int main(void)
{
    printf("F769 Starting...\r\n");
    
    lv_init();
    tft_init();
    touchpad_init();

    // Ensure the default screen is loaded and active
    lv_obj_t *scr = lv_scr_act();
    lv_scr_load(scr);

    printf("Creating UI...\r\n");
    create_simple_ui();
    printf("UI created successfully\r\n");

    update_ui();
    printf("Initial UI update complete\r\n");

    ethernet = EthInterface::get_default_instance();
    if (!ethernet) {
        printf("No Ethernet interface found\r\n");
        return -1;
    }
    
    printf("Connecting to network...\r\n");
    int ret = ethernet->connect();
    if (ret != 0) {
        printf("Network connection failed: %d\r\n", ret);
        return -1;
    }
    
    printf("Network connected\r\n");
    
    NetworkInterface *net = ethernet;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    global_client = &client;
    
    printf("Connecting to MQTT broker at %s:1883\r\n", MQTT_HOST);
    int rc = mqttNetwork.connect(MQTT_HOST, 1883);
    if (rc != 0) {
        printf("MQTT network connection failed: %d\r\n", rc);
        return -1;
    }
    
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char *)"F769_TouchController";
    
    rc = client.connect(data);
    if (rc != 0) {
        printf("MQTT client connection failed: %d\r\n", rc);
        return -1;
    }
    
    printf("MQTT connected\r\n");
    
    // Subscribe to all topics
    rc = client.subscribe(CAR_DISTANCE_TOPIC, MQTT::QOS0, distance_messageArrived);
    if (rc != 0) {
        printf("MQTT distance subscription failed: %d\r\n", rc);
        return -1;
    }
    
    rc = client.subscribe(CAR_PATTERN_TOPIC, MQTT::QOS0, pattern_messageArrived);
    if (rc != 0) {
        printf("MQTT pattern subscription failed: %d\r\n", rc);
        return -1;
    }
    
    rc = client.subscribe(CAR_PING_TOPIC, MQTT::QOS0, ping_messageArrived);
    if (rc != 0) {
        printf("MQTT ping subscription failed: %d\r\n", rc);
        return -1;
    }
    
    rc = client.subscribe(CAR_STATE_TOPIC, MQTT::QOS0, state_messageArrived);
    if (rc != 0) {
        printf("MQTT state subscription failed: %d\r\n", rc);
        return -1;
    }
    
    printf("F769 subscribed to: %s, %s, %s, and %s\r\n", CAR_DISTANCE_TOPIC, CAR_PATTERN_TOPIC, CAR_PING_TOPIC, CAR_STATE_TOPIC);
    printf("F769 publishing to: %s\r\n", CAR_CONTROL_TOPIC);

    // Start only MQTT thread for handling MQTT operations
    printf("Starting MQTT thread...\r\n");
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    
    // Schedule periodic MQTT yield
    mqtt_queue.call_every(chrono::milliseconds(MQTT_YIELD_INTERVAL_MS), callback(&mqtt_yield_task));
    
    // Schedule periodic MQTT connection check every 10 seconds
    mqtt_queue.call_every(chrono::milliseconds(MQTT_CONNECTION_CHECK_MS), callback(&mqtt_connection_check));
    
    printf("Entering main loop...\r\n");
    
    // Main loop handles LVGL and UI updates
    int ui_update_counter = 0;
    while (1)
    {
        lv_tick_inc(LVGL_TICK_INTERVAL_MS);
        lv_task_handler();

        // Check for pending UI updates and handle them in main thread
        if (distance_update_pending || pattern_update_pending || ping_update_pending || state_update_pending) {
            update_ui();
            distance_update_pending = false;
            pattern_update_pending = false;
            ping_update_pending = false;
            state_update_pending = false;
        }
        
        // Periodic UI refresh
        ui_update_counter++;
        if (ui_update_counter >= UI_UPDATE_CYCLES) {  // Update every 2 seconds
            char dist_str[64];
            sprintf(dist_str, "Total Distance: %.1f cm", car_distance);
            lv_label_set_text(distance_label, dist_str);
            
            char ping_str[64];
            sprintf(ping_str, "LaserPing: %.1f cm", car_ping_distance);
            lv_label_set_text(ping_label, ping_str);
            
            ui_update_counter = 0;
        }

        ThisThread::sleep_for(chrono::milliseconds(LVGL_TICK_INTERVAL_MS));
        led = !led;
    }
}