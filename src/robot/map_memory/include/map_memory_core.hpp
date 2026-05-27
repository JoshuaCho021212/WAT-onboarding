#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
namespace robot
{
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void integrateCostmap(
      const nav_msgs::msg::OccupancyGrid& costmap, 
      double robot_x, 
      double robot_y,
      double robot_yaw);
    nav_msgs::msg::OccupancyGrid getGlobalMap();
  private:
    rclcpp::Logger logger_;   
    nav_msgs::msg::OccupancyGrid global_map_;
};
}  
#endif
