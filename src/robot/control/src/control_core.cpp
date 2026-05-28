#include "control_core.hpp"

namespace robot
{

  void ControlCore::setPath(nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
  }

  void ControlCore::setOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odometry_ = msg;
  }

  geometry_msgs::msg::Twist ControlCore::computeVelocity() {
    geometry_msgs::msg::Twist cmd;
    return cmd;
  }

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    lookahead_distance_ = 1.0;
    goal_tolerance_     = 0.1;
    linear_speed_       = 0.5;
  }

}  
