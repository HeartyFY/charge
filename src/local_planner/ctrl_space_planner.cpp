#include <tf/tf.h>

#include "ctrl_space_planner.h"

namespace local_planner
{

CtrlSpacePlanner::CtrlSpacePlanner(std::shared_ptr<RobotModel> robot_model) 
  : robot_model_(robot_model), planned_trajectory_(std::make_shared<plan_interface::Trajectory>()) {}

std::shared_ptr<plan_interface::Trajectory> CtrlSpacePlanner::get_result() {
  return planned_trajectory_;
}

void CtrlSpacePlanner::feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, 
                            const ObstacleMsg& obs, const plan_interface::Path& glob_path) {
  current_map_ = map;
  current_odom_ = odom;
  current_obs_ = obs;
  glob_path_ = glob_path;
}

// bool CtrlSpacePlanner::update() {
//   if (!current_map_.data.empty()) {
//     int width = current_map_.info.width;
//     int height = current_map_.info.height;
//     double resolution = current_map_.info.resolution;

//     // Transform world coordinates to map coordinates for start and goal
//     int start_x = static_cast<int>((current_odom_.position.x - current_map_.info.origin.position.x) / resolution);
//     int start_y = static_cast<int>((current_odom_.position.y - current_map_.info.origin.position.y) / resolution);
//     State start_state(current_odom_.position.x, current_odom_.position.y, get_yaw_from_ros_q(current_odom_.orientation));
//     // int goal_x = static_cast<int>((current_goal_.pose.position.x - current_map_.info.origin.position.x) / resolution);
//     // int goal_y = static_cast<int>((current_goal_.pose.position.y - current_map_.info.origin.position.y) / resolution);

//     std::priority_queue<StateNode, std::vector<StateNode>, CompareStateNode> open_set;
//     std::unordered_map<int, StateNode *> visited;

//     auto index = [width](int x, int y){ return y * width + x; };

//     Node *start_node = new Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
//     open_set.push(*start_node);
//     visited[index(start_x, start_y)] = start_node;

//     const std::vector<std::pair<int, int>> directions{{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

//     while (!open_set.empty()) {
//       Node current = open_set.top();
//       open_set.pop();

//       if (current.x == goal_x && current.y == goal_y) {
//         planned_path_ = reconstruct_path(&current);
//         for (auto &pair : visited) {
//           delete pair.second;
//         }
//         return true;
//       }

//       for (const auto &[dx, dy] : directions) {
//         int nx = current.x + dx;
//         int ny = current.y + dy;

//         if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
//           continue;
//         }

//         if (is_collision(nx, ny)) {
//           continue;
//         }

//         double g = current.g + std::hypot(dx, dy);
//         double h = heuristic(nx, ny, goal_x, goal_y);

//         if (visited.find(index(nx, ny)) == visited.end() || g < visited[index(nx, ny)]->g) {
//           Node *neighbor = new Node(nx, ny, g, h, visited[index(current.x, current.y)]);
//           open_set.push(*neighbor);
//           visited[index(nx, ny)] = neighbor;
//         }
//       }
//     }

//     for (auto &pair : visited) {
//       delete pair.second;
//     }
//     return false;
//   }
//   return false;
// }



} // namespace local_planner