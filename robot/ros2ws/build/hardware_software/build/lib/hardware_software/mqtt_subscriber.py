import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
from nav_msgs.msg import OccupancyGrid
import pickle

class Mqtt2RosBridge(Node):
    def __init__(self):
        super().__init__('mqtt2ros_bridge')
        self.publisher = self.create_publisher(OccupancyGrid, '/map_from_mqtt', 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_mqtt_message
        # self.mqtt_client.connect("192.168.1.59", 1883, 60)  # Use your broker IP
        # self.mqtt_client.connect("192.168.43.204", 1883, 60)
        self.mqtt_client.connect("192.168.43.56", 1883, 60)
        self.mqtt_client.subscribe("map")
        self.mqtt_client.loop_start()

    # def on_mqtt_message(self, client, userdata, msg):
    #     ros_msg = OccupancyGrid()
    #     ros_msg.header.stamp = self.get_clock().now().to_msg()
    #     ros_msg.header.frame_id = "map"
    #     ros_msg.data = list(msg.payload)
    #     self.get_logger().info(f'Received from MQTT: "{ros_msg.header}"')
    #     self.publisher.publish(ros_msg)
    def on_mqtt_message(self, client, userdata, msg):
        ros_msg = pickle.loads(msg.payload)
        self.get_logger().info(f'Received from MQTT: "{ros_msg.header}"')
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