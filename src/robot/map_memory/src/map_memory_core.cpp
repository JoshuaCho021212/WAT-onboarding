#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{
MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    global_map_.header.frame_id = "sim_world";
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 200;
    global_map_.info.height = 200;
    global_map_.info.origin.position.x = -10.0;
    global_map_.info.origin.position.y = -10.0;
    global_map_.data.assign(200 * 200, -1);
}

void MapMemoryCore::integrateCostmap(
  const nav_msgs::msg::OccupancyGrid& costmap,
  double robot_x, double robot_y, double robot_yaw) {
    for (int cy = 0; cy < (int)costmap.info.height; cy++) {
      for (int cx = 0; cx < (int)costmap.info.width; cx++) {
        int idx = cy * costmap.info.width + cx;
        int8_t val = costmap.data[idx];
        if (val == -1) continue;

        double local_x = (cx + 0.5) * costmap.info.resolution + costmap.info.origin.position.x;
        double local_y = (cy + 0.5) * costmap.info.resolution + costmap.info.origin.position.y;

        double global_x = robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
        double global_y = robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

        int gx = (int)((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int gy = (int)((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

        if (gx < 0 || gx >= (int)global_map_.info.width ||
            gy < 0 || gy >= (int)global_map_.info.height) continue;

        global_map_.data[gy * global_map_.info.width + gx] = val;
      }
    }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() {
  return global_map_;
}
}
