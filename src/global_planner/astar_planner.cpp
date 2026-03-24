#include <queue>
#include <cmath>
#include <algorithm>

#include "geometry.h"
#include "astar_planner.h"

namespace global_planner {

AstarPlanner::AstarPlanner(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & global_config_json) 
  : robot_model_(std::make_shared<RobotModel>(*robot_model)), planned_path_(std::make_shared<plan_interface::Path>()) {
  parse_config_json(global_config_json);

  // for debug
  init_mk();
  // for debug
}

void AstarPlanner::parse_config_json(const nlohmann::json & global_config_json) {
  cfg_.front_expansion = global_config_json.contains("front_expansion") ? global_config_json.at("front_expansion").get<double>() : 0.15;
  cfg_.bounding_radius = (robot_model_->get_length() + cfg_.front_expansion) / 2;

  std::cout << "AstarPlanner initialized" << std::endl;
  std::cout << "cfg_.front_expansion: " << cfg_.front_expansion << std::endl;
  std::cout << "cfg_.bounding_radius = " << cfg_.bounding_radius << std::endl;
}

double AstarPlanner::heuristic(int x1, int y1, int x2, int y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

bool AstarPlanner::is_glob_collision(int gx, int gy) {
  int bounding_radius_cells = static_cast<int>(std::ceil(cfg_.bounding_radius / current_map_.info.resolution));

  for (int dx = -bounding_radius_cells; dx <= bounding_radius_cells; ++dx) {
    for (int dy = -bounding_radius_cells; dy <= bounding_radius_cells; ++dy) {
      int nx = gx + dx;
      int ny = gy + dy;

      if (!robot_model_->is_within_glob_map_bounds(nx, ny)) {
        return true;
      }

      if (current_map_.data[ny * current_map_.info.width + nx] > 0) {

        // for debug
        auto [wx, wy] = robot_model_->glob_map_to_world_coords(nx, ny);
        geometry_msgs::Point p;
        p.x = wx;
        p.y = wy;
        global_collision_mk.points.push_back(p);
        // for debug

        return true;
      }
    }
  }
  return false;
}

bool AstarPlanner::is_loc_collision(int lx, int ly) {
  if (current_obs_.data.empty()) {
    return false;
  }

  int bounding_radius_cells = static_cast<int>(std::ceil(cfg_.bounding_radius / current_obs_.info.resolution));

  for (int dx = -bounding_radius_cells; dx <= bounding_radius_cells; ++dx) {
    for (int dy = -bounding_radius_cells; dy <= bounding_radius_cells; ++dy) {
      int nx = lx + dx;
      int ny = ly + dy;

      int local_width = current_obs_.info.width;
      int local_height = current_obs_.info.height;

      if (!robot_model_->is_within_loc_map_bounds(nx, ny)) {
        return false;
      }
      
      if (current_obs_.data[ny * current_obs_.info.width + nx].occupancy) {

        // for debug
        auto [wx, wy] = robot_model_->loc_map_to_world_coords(nx, ny);
        geometry_msgs::Point p;
        p.x = wx;
        p.y = wy;
        local_collision_mk.points.push_back(p);
        // for debug

        return true;
      }
    }
  }
  return false;
}

bool AstarPlanner::is_collision(int gx, int gy) {
  if (is_glob_collision(gx, gy)) {
    return true;
  } else {
    auto [wx, wy] = robot_model_->glob_map_to_world_coords(gx, gy);
    auto [lx, ly] = robot_model_->world_to_loc_map_coords(wx, wy);
    if (robot_model_->is_within_valid_loc_map_bounds(lx, ly)) {
      return is_loc_collision(lx, ly);
    } else {
      return false;
    }
  }

  // auto [wx, wy] = golb_map_to_world_coords(gx, gy);
  // auto [lx, ly] = world_to_loc_map_coords(wx, wy);
  // if (is_within_valid_loc_map_bounds(lx, ly)) {
  //   return is_loc_collision(lx, ly);
  // } else {
  //   return is_glob_collision(gx, gy);
  // }
}
// for debug
void AstarPlanner::init_mk() {
  std_msgs::ColorRGBA red; // rgb(255, 0, 0)
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 0.5;

  global_collision_mk.header.frame_id  = local_collision_mk.header.frame_id = "map";
  global_collision_mk.id = 0;
  local_collision_mk.id = 1;
  global_collision_mk.action = local_collision_mk.action = visualization_msgs::Marker::ADD;
  global_collision_mk.type = local_collision_mk.type = visualization_msgs::Marker::CUBE_LIST;
  global_collision_mk.pose.orientation.w = local_collision_mk.pose.orientation.w = 1.0;
  global_collision_mk.scale.x = 0.1 * 0.8;
  global_collision_mk.scale.y = 0.1 * 0.8;
  global_collision_mk.scale.z = 0.1 * 0.1;
  local_collision_mk.scale.x = 0.05 * 0.8;
  local_collision_mk.scale.y = 0.05 * 0.8;
  local_collision_mk.scale.z = 0.05 * 0.1;

  global_collision_mk.color = local_collision_mk.color = red;
}
// for debug
// for debug
visualization_msgs::MarkerArray AstarPlanner::get_collision_mks() {
  collision_mks.markers.clear();
  visualization_msgs::Marker deleter;
  deleter.action = visualization_msgs::Marker::DELETEALL;
  collision_mks.markers.emplace_back(deleter);
  if (!global_collision_mk.points.empty()) {
    collision_mks.markers.emplace_back(global_collision_mk);
  }
  if (!local_collision_mk.points.empty()) {
    collision_mks.markers.emplace_back(local_collision_mk);
  }
  return collision_mks;
}
// for debug

std::shared_ptr<plan_interface::Path> AstarPlanner::reconstruct_path(Node *goal_node) {
  auto path = std::make_shared<plan_interface::Path>();

  Node *current = goal_node;
  plan_interface::PathPoint goal_point;
  goal_point.x = current_goal_.pose.position.x;
  goal_point.y = current_goal_.pose.position.y;
  goal_point.yaw = utils::geometry::get_yaw_from_ros_q(current_goal_.pose.orientation);
  path->path_points.push_back(goal_point);
  current = current->parent;

  while (current) {
    plan_interface::PathPoint point;

    auto [wx, wy] = robot_model_->glob_map_to_world_coords(current->x, current->y);
    point.x = wx;
    point.y = wy;
    point.yaw = std::atan2(path->path_points.back().y - point.y, path->path_points.back().x - point.x);

    path->path_points.push_back(point);
    current = current->parent;
  }

  std::reverse(path->path_points.begin(), path->path_points.end());
  path->length = static_cast<double>(path->path_points.size());
  path->segments = static_cast<int>(path->path_points.size()) - 1;
  return path;
}

bool AstarPlanner::update() {
  if (!current_map_.data.empty()) {

    global_collision_mk.points.clear();
    local_collision_mk.points.clear();

    glob_map_origin_[0] = current_map_.info.origin.position.x;
    glob_map_origin_[1] = current_map_.info.origin.position.y;
    glob_map_origin_[2] = utils::geometry::get_yaw_from_ros_q(current_map_.info.origin.orientation);
    int width = current_obs_.info.width;
    int height = current_obs_.info.height;
    double resolution = current_obs_.info.resolution;
    loc_map_origin_[2] = utils::geometry::get_yaw_from_ros_q(current_obs_.info.origin.orientation);
    loc_map_origin_[0] = current_obs_.info.origin.position.x - (std::cos(loc_map_origin_[2])*width*resolution/2 - std::sin(loc_map_origin_[2])*height*resolution/2);
    loc_map_origin_[1] = current_obs_.info.origin.position.y - (std::sin(loc_map_origin_[2])*width*resolution/2 + std::cos(loc_map_origin_[2])*height*resolution/2);
    

    auto [start_x, start_y] = robot_model_->world_to_glob_map_coords(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y);
    auto [goal_x, goal_y] = robot_model_->world_to_glob_map_coords(current_goal_.pose.position.x, current_goal_.pose.position.y);

    std::priority_queue<Node, std::vector<Node>, CompareNode> open_set;
    std::unordered_map<int, Node *> visited;

    auto index = [width = current_map_.info.width](int x, int y){ return y * width + x; };

    Node *start_node = new Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
    open_set.push(*start_node);
    visited[index(start_x, start_y)] = start_node;

    const std::vector<std::pair<int, int>> directions{{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!open_set.empty()) {
      Node current = open_set.top();
      open_set.pop();

      if (current.x == goal_x && current.y == goal_y) {
        planned_path_ = reconstruct_path(&current);
        for (auto &pair : visited) {
          delete pair.second;
        }
        return true;
      }

      for (const auto &[dx, dy] : directions) {
        int nx = current.x + dx;
        int ny = current.y + dy;

        if (!robot_model_->is_within_glob_map_bounds(nx, ny)) {
          continue;
        }

        if (is_collision(nx, ny)) {
          continue;
        }

        double g = current.g + std::hypot(dx, dy);
        double h = heuristic(nx, ny, goal_x, goal_y);

        if (visited.find(index(nx, ny)) == visited.end() || g < visited[index(nx, ny)]->g) {
          Node *neighbor = new Node(nx, ny, g, h, visited[index(current.x, current.y)]);
          open_set.push(*neighbor);
          visited[index(nx, ny)] = neighbor;
        }
      }
    }

    for (auto &pair : visited) {
      delete pair.second;
    }
    return false;
  }
  return false;
}

std::shared_ptr<plan_interface::Path> AstarPlanner::get_result() {
  return planned_path_;
}

void AstarPlanner::feed_data(const MapInfoMsg &map, const OdomInfoMsg &odom, const ObstacleMsg &obs,
                            const navi_types::Waypoint &goal, const plan_interface::Gear& gear) {
  current_map_ = map;
  current_odom_ = odom;
  current_goal_ = goal;
  current_obs_ = obs;

  MapMeta glob_map_meta;
  glob_map_meta.width = current_map_.info.width;
  glob_map_meta.height = current_map_.info.height;
  glob_map_meta.resolution = current_map_.info.resolution;
  glob_map_meta.origin_x = current_map_.info.origin.position.x;
  glob_map_meta.origin_y = current_map_.info.origin.position.y;
  glob_map_meta.rotation = utils::geometry::get_yaw_from_ros_q(current_map_.info.origin.orientation);
  robot_model_->set_glob_map_meta(glob_map_meta);
  
  int width = current_obs_.info.width;
  int height = current_obs_.info.height;
  double resolution = current_obs_.info.resolution;
  double rotation = utils::geometry::get_yaw_from_ros_q(current_obs_.info.origin.orientation);
  MapMeta loc_map_meta;
  loc_map_meta.width = width;
  loc_map_meta.height = height;
  loc_map_meta.resolution = resolution;
  loc_map_meta.origin_x = current_obs_.info.origin.position.x - (std::cos(rotation)*width*resolution/2 - std::sin(rotation)*height*resolution/2);
  loc_map_meta.origin_y = current_obs_.info.origin.position.y - (std::sin(rotation)*width*resolution/2 + std::cos(rotation)*height*resolution/2);
  loc_map_meta.rotation = rotation;
  robot_model_->set_loc_map_meta(loc_map_meta);
}

} // namespace global_planner