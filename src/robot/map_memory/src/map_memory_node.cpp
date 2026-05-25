#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 200;
  global_map_.info.height = 200;
  global_map_.info.origin.position.x = -10.0;
  global_map_.info.origin.position.y = -10.0;
  global_map_.data.assign(200 * 200, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  if (distance >= 1.5) {
    robot_x_ = x;
    robot_y_ = y;

    auto q = msg->pose.pose.orientation;
    robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_x_ = x;
    last_y_ = y;
    should_update_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_ && costmap_received_) {
    map_memory_.integrateCostmap(latest_costmap_, global_map_, robot_x_, robot_y_, robot_yaw_);
    map_pub_->publish(global_map_);
    should_update_ = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
