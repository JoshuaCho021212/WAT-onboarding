#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    void setPath(nav_msgs::msg::Path::SharedPtr);
    void setOdom(nav_msgs::msg::Odometry::SharedPtr);
    geometry_msgs::msg::Twist computeVelocity();
  
  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odometry_;
    double lookahead_distance_, goal_tolerance_, linear_speed_ = 0.0;
};
} 

#endif 
