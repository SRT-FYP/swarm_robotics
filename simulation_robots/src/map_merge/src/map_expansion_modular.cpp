#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <algorithm>

using std::placeholders::_1;

class MapExpansionNode : public rclcpp::Node
{
public:
  MapExpansionNode() : Node("map_expansion_node")
  {
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(50))
              .transient_local()
              .reliable();

    // Timer for discovering new map topics
    discovery_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapExpansionNode::discover_map_topics, this));

    // Main processing timer
    processing_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MapExpansionNode::process_maps, this));
  }

private:
  void discover_map_topics()
  {
    // Get all current topics
    auto topic_map = this->get_topic_names_and_types();

    // Find all map topics (ends with /map)
    for (const auto& [topic_name, topic_types] : topic_map) {
      if (topic_name.find("/map") != std::string::npos && 
          topic_name.rfind("/map") == topic_name.length() - 4) {
        
        // Extract namespace (everything before /map)
        std::string ns = topic_name.substr(0, topic_name.length() - 4);
        
        // If we haven't seen this namespace before, create subscriber and publisher
        if (subscribers_.find(ns) == subscribers_.end()) {
          RCLCPP_INFO(this->get_logger(), "Discovered new map topic in namespace: %s", ns.c_str());
          
          // Create subscriber for this namespace's map
          auto sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic_name, 10, 
            [this, ns](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
              this->mapCallback(ns, *msg);
            });
          
          // Create publisher for expanded map
          auto pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            ns + "/expanded_map", 
            rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable());
          
          // Store them
          subscribers_[ns] = sub;
          publishers_[ns] = pub;
          maps_[ns] = nav_msgs::msg::OccupancyGrid(); // Initialize empty map
        }
      }
    }
  }

  void mapCallback(const std::string& ns, const nav_msgs::msg::OccupancyGrid& msg)
  {
    maps_[ns] = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received map update for namespace: %s", ns.c_str());
  }

  void process_maps()
  {
    for (auto& [ns, map] : maps_) {
      if (!map.data.empty()) {
        // Create expanded map
        auto expanded_map = create_expanded_map(map, ns + "_map");
        
        // Publish expanded map
        if (publishers_.find(ns) != publishers_.end()) {
          publishers_[ns]->publish(expanded_map);
          RCLCPP_DEBUG(this->get_logger(), "Published expanded map for namespace: %s", ns.c_str());
        }
      }
    }
  }

  nav_msgs::msg::OccupancyGrid create_expanded_map(const nav_msgs::msg::OccupancyGrid& original_map, const std::string& frame_id)
  {
    nav_msgs::msg::OccupancyGrid new_map;
    
    // Set basic map properties
    new_map.header.frame_id = frame_id;
    new_map.header.stamp = this->now();
    new_map.info.resolution = 0.05;
    new_map.info.origin.position.x = -10.0;
    new_map.info.origin.position.y = -10.0;
    new_map.info.origin.position.z = 0.0;
    new_map.info.origin.orientation.w = 1.0;
    
    // Set map dimensions
    const size_t width = 384;
    const size_t height = 384;
    new_map.info.width = width;
    new_map.info.height = height;

    // Calculate offset between new map and original map
    const size_t bottom_width = static_cast<size_t>(
      (original_map.info.origin.position.x - new_map.info.origin.position.x) / new_map.info.resolution);
    const size_t bottom_height = static_cast<size_t>(
      (original_map.info.origin.position.y - new_map.info.origin.position.y) / new_map.info.resolution);

    // Fill the map with unknown cells (-1)
    new_map.data.resize(width * height, -1);

    // If original map has data, copy it to the correct position
    if (!original_map.data.empty())
    {
      for (size_t y = 0; y < original_map.info.height; ++y)
      {
        for (size_t x = 0; x < original_map.info.width; ++x)
        {
          size_t new_x = x + bottom_width;
          size_t new_y = y + bottom_height;
          
          if (new_x < width && new_y < height)
          {
            size_t new_index = new_y * width + new_x;
            size_t orig_index = y * original_map.info.width + x;
            new_map.data[new_index] = original_map.data[orig_index];
          }
        }
      }
    }

    return new_map;
  }

  // Member variables
  std::map<std::string, nav_msgs::msg::OccupancyGrid> maps_;
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> publishers_;
  std::map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subscribers_;
  
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapExpansionNode>());
  rclcpp::shutdown();
  return 0;
}