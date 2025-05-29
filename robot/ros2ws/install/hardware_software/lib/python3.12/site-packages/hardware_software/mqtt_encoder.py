# ros2_mqtt_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
from nav_msgs.msg import OccupancyGrid
import pickle

class Ros2MqttBridge(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')
        self.mqtt_client = mqtt.Client()
        # self.mqtt_client.connect("192.168.1.59", 1883, 60)  # Replace with broker IP
        # self.mqtt_client.connect("192.168.43.204", 1883, 60)
        self.mqtt_client.connect("192.168.43.56", 1883, 60)
        self.mqtt_client.loop_start()

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10
        )

    # def listener_callback(self, msg):
    #     self.get_logger().info(f'Publishing to MQTT: "{msg.header}"')
    #     self.mqtt_client.publish("map",  f"{msg.header}")

    def listener_callback(self, msg):
        self.get_logger().info(f'Publishing to MQTT: "{msg.header}"')
        payload = pickle.dumps(msg)
        self.mqtt_client.publish("map", payload)

def main(args=None):
    rclpy.init(args=args)
    bridge = Ros2MqttBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
