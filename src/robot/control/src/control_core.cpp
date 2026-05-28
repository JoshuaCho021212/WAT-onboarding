#include "control_core.hpp"
#include <cmath>

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
    if(!current_path_ || current_path_->poses.empty()) {
    return cmd;
    }

    if(!current_odometry_) {
      return cmd;
    }

  auto& robot_pos = current_odometry_->pose.pose.position;
  auto& goal_pos  = current_path_->poses.back().pose.position;
  double gdx = goal_pos.x - robot_pos.x;
  double gdy = goal_pos.y - robot_pos.y;
  double dist = std::sqrt(gdx*gdx + gdy*gdy);
    
  if(dist < goal_tolerance_) {
      return cmd;
  }

  geometry_msgs::msg::Point lookahead_point;
  for(const auto& pose : current_path_->poses) {
    double ldx = pose.pose.position.x - robot_pos.x;
    double ldy = pose.pose.position.y - robot_pos.y;
    double ldist = std::sqrt(ldx*ldx + ldy*ldy);
    
    if(ldist >= lookahead_distance_) {
        lookahead_point = pose.pose.position;
        break; 
    }
  }

  auto& quat = current_odometry_->pose.pose.orientation;

  double yaw = std::atan2(
    2.0 * (quat.w * quat.z + quat.x * quat.y),
    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));

  double pdx = lookahead_point.x - robot_pos.x;
  double pdy = lookahead_point.y - robot_pos.y;

  double local_y = -(pdx) * std::sin(yaw) + (pdy) * std::cos(yaw);

  double L = std::sqrt(pdx*pdx + pdy*pdy);
  double curvature = 2.0 * local_y / (L * L);
  cmd.linear.x = linear_speed_;
  cmd.angular.z = linear_speed_ * curvature;
    
  return cmd;
  }

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    lookahead_distance_ = 1.0;
    goal_tolerance_     = 0.1;
    linear_speed_       = 0.5;
  }

}  
