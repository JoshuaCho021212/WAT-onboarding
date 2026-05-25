#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot {

class CostmapCore {
public:
  CostmapCore(const rclcpp::Logger& logger);

  void initializeCostmap();
  void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  nav_msgs::msg::OccupancyGrid publishCostmap();

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::OccupancyGrid costmap_;
  std::vector<std::vector<int>> grid_;
  int width_ = 100;
  int height_ = 100;
  double resolution_ = 0.1;
};

}

#endif
