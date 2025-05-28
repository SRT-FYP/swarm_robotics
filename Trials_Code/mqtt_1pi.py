import paho.mqtt.client as mqtt

# Configuration for local broker
broker = "192.168.43.204"     # or use "127.0.0.1"
port = 1883
topic = "my/topic"
message = "Hello from Raspberry Pi!"

# Create MQTT client instance
client = mqtt.Client()

# Connect to Mosquitto broker
client.connect(broker, port, 60)

# Publish the message
client.publish(topic, message)

# Disconnect cleanly
client.disconnect()

print(f"Published '{message}' to topic '{topic}'")
