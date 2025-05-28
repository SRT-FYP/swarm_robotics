//source: https://github.com/gingineer95/Multi-Robot-Exploration-and-Map-Merging/blob/main/src/map_expansion.cpp
// but updated to work with ROS2

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <memory>

using std::placeholders::_1;

class MapExpansionNode : public rclcpp::Node
{
public:
  MapExpansionNode() : Node("map_expansion_node")
  {
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(50))
              .transient_local()
              .reliable();
    // Create publishers
    new_leader_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/leader/expanded_map", map_qos);
    new_follower_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/follower/expanded_map", map_qos);

    // Create subscribers
    map_meta0_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "leader/map", 10, std::bind(&MapExpansionNode::map0Callback, this, _1));
    
    map_meta1_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "follower/map", 10, std::bind(&MapExpansionNode::map1Callback, this, _1));

    // Main processing timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&MapExpansionNode::process_maps, this));
  }

private:
  void map0Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    slam0_map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received leader map");
  }

  void map1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    slam1_map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received follower map");
  }

  void process_maps()
  {
    if (!slam0_map_.data.empty() || !slam1_map_.data.empty() || 
        slam0_map_.info.origin.position.x != 0 || slam1_map_.info.origin.position.x != 0)
    {
      // Create new expanded maps
      auto new_leader_map = create_expanded_map(slam0_map_, "map");
      // auto new_leader_map = create_expanded_map(slam0_map_, "new_leader_map");
      auto new_follower_map = create_expanded_map(slam1_map_, "map");
      // auto new_leader_map = create_expanded_map(slam0_map_, "new_follower_map");

      // Publish the new maps
      new_leader_map_pub_->publish(new_leader_map);
      new_follower_map_pub_->publish(new_follower_map);
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
    new_map.info.origin.orientation.w = 1.0;  // Changed from 0.0 to valid quaternion
    
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
  nav_msgs::msg::OccupancyGrid slam0_map_;
  nav_msgs::msg::OccupancyGrid slam1_map_;
  
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr new_leader_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr new_follower_map_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_meta0_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_meta1_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapExpansionNode>());
  rclcpp::shutdown();
  return 0;
}