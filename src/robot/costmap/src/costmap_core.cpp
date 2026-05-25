#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap() {
  grid_ = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>(x / resolution_) + width_ / 2;
  y_grid = static_cast<int>(y / resolution_) + height_ / 2;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = 100;
  }
}

void CostmapCore::inflateObstacles() {
  int inflation_cells = static_cast<int>(1.0 / resolution_); 
  auto grid_copy = grid_;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_copy[y][x] == 100) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && distance <= 1.0) {
              int cost = static_cast<int>(100 * (1 - distance / 1.0));
              if (cost > grid_[ny][nx]) {
                grid_[ny][nx] = cost;
              }
            }
          }
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::publishCostmap() {
  costmap_.info.resolution = resolution_;
  costmap_.info.width = width_;
  costmap_.info.height = height_;
  costmap_.data.clear();
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      costmap_.data.push_back(grid_[y][x]);
    }
  }
  return costmap_;
}

}