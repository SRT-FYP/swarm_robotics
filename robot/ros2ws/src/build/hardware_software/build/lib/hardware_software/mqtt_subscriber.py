import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class Mqtt2RosBridge(Node):
    def __init__(self):
        super().__init__('mqtt2ros_bridge')
        self.publisher = self.create_publisher(String, '/map_from_mqtt', 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("192.168.43.204", 1883, 60)  # Use your broker IP
        self.mqtt_client.subscribe("map")
        self.mqtt_client.loop_start()

    def on_mqtt_message(self, client, userdata, msg):
        ros_msg = String()
        ros_msg.data = msg.payload.decode()
        self.get_logger().info(f'Received from MQTT: "{ros_msg.data}"')
        self.publisher.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = Mqtt2RosBridge()
    try:
        rclpy.spin(bridge)
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()