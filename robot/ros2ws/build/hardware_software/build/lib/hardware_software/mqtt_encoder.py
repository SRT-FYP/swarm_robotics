# ros2_mqtt_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class Ros2MqttBridge(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("192.168.43.204", 1883, 60)  # Replace with broker IP
        self.mqtt_client.loop_start()

        self.subscription = self.create_subscription(
            String,
            'robot1/chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Publishing to MQTT: "{msg.data}"')
        self.mqtt_client.publish("robot1/chatter", msg.data)

def main(args=None):
    rclpy.init(args=args)
    bridge = Ros2MqttBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
