import paho.mqtt.client as paho
import time

# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client

# MQTT broker hosted on local machine
mqttc = paho.Client()

# Settings for connection
# TODO: revise host to your IP
host = "192.168.1.44"
topic = "Mbed"

# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    payload = msg.payload.decode('utf-8')
    print("[Received] Topic: " + msg.topic + ", Message: " + payload)
    
    # Parse accelerometer data if present
    if "Accelerometer:" in payload:
        try:
            # Extract values
            parts = payload.split("X=")[1].split(", Y=")
            x_val = float(parts[0])
            y_val = float(parts[1].split(", Z=")[0])
            z_val = float(parts[1].split(", Z=")[1])
            
            print(f"\n=== ACCELEROMETER READINGS ===")
            print(f"X: {x_val:.6f}")
            print(f"Y: {y_val:.6f}")
            print(f"Z: {z_val:.6f}")
            print(f"=============================\n")
        except Exception as e:
            print(f"Error parsing accelerometer data: {e}")

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

# Loop forever, receiving messages
mqttc.loop_forever()
