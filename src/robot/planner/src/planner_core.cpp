#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::PointStamped& goal)
{
    CellIndex start_idx(
        (int)((start.position.x - map.info.origin.position.x) / map.info.resolution),
        (int)((start.position.y - map.info.origin.position.y) / map.info.resolution));

    CellIndex goal_idx(
        (int)((goal.point.x - map.info.origin.position.x) / map.info.resolution),
        (int)((goal.point.y - map.info.origin.position.y) / map.info.resolution));

  
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
    std::unordered_set<CellIndex, CellIndexHash> closed_list;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    open_list.push(AStarNode(start_idx, 0.0));
    g_score[start_idx] = 0.0;

  
    while (!open_list.empty()) {
        CellIndex current = open_list.top().index;
        open_list.pop();

        if (current == goal_idx) break;

        if (closed_list.count(current)) continue;
        closed_list.insert(current);

        std::vector<CellIndex> neighbors = {
            {current.x + 1, current.y},
            {current.x - 1, current.y},
            {current.x, current.y + 1},
            {current.x, current.y - 1}
        };

        for (auto& neighbor : neighbors) {
            if (neighbor.x < 0 || neighbor.x >= (int)map.info.width ||
                neighbor.y < 0 || neighbor.y >= (int)map.info.height) continue;

            int idx = neighbor.y * map.info.width + neighbor.x;
            if (map.data[idx] > 50) continue;

            double g = g_score[current] + 1.0;
            double h = std::sqrt(std::pow(neighbor.x - goal_idx.x, 2) +
                                 std::pow(neighbor.y - goal_idx.y, 2));
            double f = g + h;

            if (!g_score.count(neighbor) || g < g_score[neighbor]) {
                g_score[neighbor] = g;
                came_from[neighbor] = current;
                open_list.push(AStarNode(neighbor, f));
            }
        }
    }

  
    if (came_from.find(goal_idx) == came_from.end()) {
        RCLCPP_WARN(logger_, "No path found!");
        return nav_msgs::msg::Path();
    }

    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "sim_world";

    CellIndex current = goal_idx;
    while (current != start_idx) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = current.x * map.info.resolution + map.info.origin.position.x;
        pose.pose.position.y = current.y * map.info.resolution + map.info.origin.position.y;
        path.poses.push_back(pose);
        current = came_from[current];
    }

    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

}
