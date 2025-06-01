import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import json
import paho.mqtt.client as mqtt


class MapToJsonNode(Node):
    def __init__(self):
        super().__init__('map_to_json_node')
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("192.168.43.204", 1883, 60)
        self.mqtt_client.loop_start()

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map_from_mqtt',  # Change topic name if needed
            self.map_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # self.json_map_publisher = self.create_publisher(
        #     OccupancyGrid,
        #     '/map_from_mqtt_json',  # Change topic name if needed
        #     10
        # )

    def map_callback(self, msg):
        map_dict = {
            "info": {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution
            },
            "data": list(msg.data)
        }

        json_output = json.dumps(map_dict, indent=2)
        
        # Print to terminal
        print(json_output)

        self.mqtt_client.publish("map_from_mqtt_json", json_output)

        self.get_logger().info('published json map to mqtt')
        # Publish the JSON map as a string message
        # self.json_map_publisher.publish(json_output)
        # # Optional: write to file
        # with open("map_output.json", "w") as f:
        #     f.write(json_output)


def main(args=None):
    rclpy.init(args=args)
    node = MapToJsonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
