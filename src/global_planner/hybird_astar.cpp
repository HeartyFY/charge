#include <limits>

#include "utils.h"
#include "geometry.h"
#include "hybird_astar.h"

namespace global_planner {

namespace hybird_astar{

HybirdAstar::HybirdAstar(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & global_config_json)
  : robot_model_(std::make_shared<RobotModel>(*robot_model)), 
    planned_path_(std::make_shared<plan_interface::Path>()) {
  parse_config_json(global_config_json);
  grid_ = std::make_shared<GridMap>();
  obst_kdtree_ = std::make_shared<utils::kdtree::KDTree2D>();
  hol_heur_ = std::make_shared<HolonomicHeuristic>(robot_model_, grid_, obst_kdtree_, cfg_.heuristic_safe_buffer);
  init_mk();
}

void HybirdAstar::parse_config_json(const nlohmann::json & global_config_json) {
  cfg_.reverse_penalty = global_config_json.contains("reverse_penalty") ? global_config_json.at("reverse_penalty").get<double>() : 1.0;
  cfg_.gear_switch_penalty = global_config_json.contains("gear_switch_penalty") ? global_config_json.at("gear_switch_penalty").get<double>() : 1.0;
  cfg_.steering_penalty = global_config_json.contains("steering_penalty") ? global_config_json.at("steering_penalty").get<double>() : 1.1;
  cfg_.steering_switch_penalty = global_config_json.contains("steering_switch_penalty") ? global_config_json.at("steering_switch_penalty").get<double>() : 1.2;
  cfg_.arc_length = global_config_json.contains("arc_length") ? global_config_json.at("arc_length").get<double>() : 0.8;
  cfg_.arc_step_length = global_config_json.contains("arc_step_length") ? global_config_json.at("arc_step_length").get<double>() : 0.1;
  cfg_.end_point_line_length = global_config_json.contains("end_point_line_length") ? global_config_json.at("end_point_line_length").get<double>() : 0.3;
  cfg_.grid_resolution = global_config_json.contains("grid_resolution") ? global_config_json.at("grid_resolution").get<double>() : 0.1;
  cfg_.angle_resolution = global_config_json.contains("angle_resolution") ? global_config_json.at("angle_resolution").get<double>()/57.3 : 10.0/57.3;
  cfg_.grid_radius_offset = global_config_json.contains("grid_radius_offset") ? global_config_json.at("grid_radius_offset").get<double>() : 5.0;
  cfg_.heuristic_safe_buffer = global_config_json.contains("heuristic_safe_buffer") ? global_config_json.at("heuristic_safe_buffer").get<double>() : 0.2;
  cfg_.search_safe_buffer = global_config_json.contains("search_safe_buffer") ? global_config_json.at("search_safe_buffer").get<double>() : 0.2;
  cfg_.charge_search_safe_buffer = global_config_json.contains("charge_search_safe_buffer") ? global_config_json.at("charge_search_safe_buffer").get<double>() : 0.1;
  cfg_.max_plan_time_ms = global_config_json.contains("max_plan_time_ms") ? global_config_json.at("max_plan_time_ms").get<uint64_t>() : 1000;

  std::cout << "HybirdAstar initialized" << std::endl;
  std::cout << "cfg_.reverse_penalty: " << cfg_.reverse_penalty << std::endl;
  std::cout << "cfg_.gear_switch_penalty: " << cfg_.gear_switch_penalty << std::endl;
  std::cout << "cfg_.steering_penalty: " << cfg_.steering_penalty << std::endl;
  std::cout << "cfg_.steering_switch_penalty: " << cfg_.steering_switch_penalty << std::endl;
  std::cout << "cfg_.arc_length: " << cfg_.arc_length << std::endl;
  std::cout << "cfg_.arc_step_length: " << cfg_.arc_step_length << std::endl;
  std::cout << "cfg_.end_point_line_length: " << cfg_.end_point_line_length << std::endl;
  std::cout << "cfg_.grid_resolution: " << cfg_.grid_resolution << std::endl;
  std::cout << "cfg_.angle_resolution: " << cfg_.angle_resolution << std::endl;
  std::cout << "cfg_.grid_radius_offset: " << cfg_.grid_radius_offset << std::endl;
  std::cout << "cfg_.heuristic_safe_buffer: " << cfg_.heuristic_safe_buffer << std::endl;
  std::cout << "cfg_.search_safe_buffer: " << cfg_.search_safe_buffer << std::endl;
  std::cout << "cfg_.charge_search_safe_buffer: " << cfg_.charge_search_safe_buffer << std::endl;
  std::cout << "cfg_.max_plan_time_ms: " << cfg_.max_plan_time_ms << std::endl;
}

std::shared_ptr<plan_interface::Path> HybirdAstar::get_result() {
  return planned_path_;
}

void HybirdAstar::feed_data(const MapInfoMsg &map, const OdomInfoMsg &odom,
                            const ObstacleMsg &obs, const navi_types::Waypoint &goal, 
                            const plan_interface::Gear& gear) {
  current_map_ = map;
  current_odom_ = odom;
  current_goal_ = goal;
  current_obs_ = obs;
  current_gear_ = gear;

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
  
  // MapMeta loc_map_meta;
  // loc_map_meta.width = current_obs_.info.width;
  // loc_map_meta.height = current_obs_.info.height;
  // loc_map_meta.resolution = current_obs_.info.resolution;
  // loc_map_meta.origin_x = current_obs_.info.origin.position.x;
  // loc_map_meta.origin_y = current_obs_.info.origin.position.y;
  // loc_map_meta.rotation = utils::geometry::get_yaw_from_ros_q(current_obs_.info.origin.orientation);
  
  robot_model_->set_loc_map_meta(loc_map_meta);
  // init_kdtree();
}

void HybirdAstar::init_kdtree() {
  uint64_t start_ts = utils::time_now_ms();
  std::vector<Eigen::Vector2d> points;
  int glob_width = current_map_.info.width;
  int glob_height = current_map_.info.height;
  for (int x = 0; x < glob_width; x++) {
    for (int y = 0; y < glob_height; y++) {
      if (x == 0 || x == glob_width-1 || y == 0 || y == glob_height-1 || (current_map_.data[y * glob_width + x] > 0)) {
        auto [wx, wy] = robot_model_->glob_map_to_world_coords(x, y);
        points.push_back(Eigen::Vector2d(wx, wy));
      }
    }
  }

  if (!current_obs_.data.empty()) {
    int local_width = current_obs_.info.width;
    int local_height = current_obs_.info.height;

    for (int x = 0; x < local_width; x++) {
      for (int y = 0; y < local_height; y++) {
        if (current_obs_.data[y * local_width + x].occupancy) {
          auto [wx, wy] = robot_model_->loc_map_to_world_coords(x, y);
          points.push_back(Eigen::Vector2d(wx, wy));
        }
      }
    }
  }
  points.push_back(Eigen::Vector2d(-1000, -1000));
  
  obst_kdtree_->init(points);

  std::cout << "kdtree build time: " << utils::time_now_ms() - start_ts << std::endl;

}

// for debug
void HybirdAstar::init_mk() {
  std_msgs::ColorRGBA red; // rgb(255, 0, 0)
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 0.3;

  expand_path_mk.header.frame_id = "map";
  expand_path_mk.ns = "expand_path";
  expand_path_mk.id = 0;
  expand_path_mk.action = visualization_msgs::Marker::ADD;
  expand_path_mk.type = visualization_msgs::Marker::LINE_LIST;
  expand_path_mk.scale.x = 0.005;
  expand_path_mk.pose.orientation.w = 1.0;
  expand_path_mk.color = red;

  std_msgs::ColorRGBA green; // rgb(25, 255, 0)
  green.a = 0.8;
  green.r = 25.0 / 255.0;
  green.g = 1.0;
  green.b = 0.0;

  second_path_mk.header.frame_id = "map";
  second_path_mk.ns = "second_path";
  second_path_mk.id = 0;
  second_path_mk.action = visualization_msgs::Marker::ADD;
  second_path_mk.type = visualization_msgs::Marker::LINE_STRIP;
  second_path_mk.scale.x = 0.015;
  second_path_mk.pose.orientation.w = 1.0;
  second_path_mk.color = green;
}
void HolonomicHeuristic::init_mk() {
  std_msgs::ColorRGBA red; // rgb(255, 0, 0)
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 0.2;

  collision_mk.header.frame_id = "map";
  collision_mk.ns = "hol_heur";
  collision_mk.id = 0;
  collision_mk.action = visualization_msgs::Marker::ADD;
  collision_mk.type = visualization_msgs::Marker::CUBE_LIST;
  collision_mk.pose.orientation.w = 1.0;
  collision_mk.scale.x = 0.1 * 0.8;
  collision_mk.scale.y = 0.1 * 0.8;
  collision_mk.scale.z = 0.1 * 0.1;
  collision_mk.color = red;
}
// for debug
// for debug
visualization_msgs::MarkerArray HybirdAstar::get_collision_mks() {
  collision_mks.markers.clear();
  visualization_msgs::Marker deleter;
  deleter.action = visualization_msgs::Marker::DELETEALL;
  collision_mks.markers.emplace_back(deleter);
  if (expand_path_mk.points.size() > 1) {
    collision_mks.markers.emplace_back(expand_path_mk);
    expand_path_mk.points.clear();
  }
  if (second_path_mk.points.size() > 1) {
    collision_mks.markers.emplace_back(second_path_mk);
    second_path_mk.points.clear();
  }

  visualization_msgs::Marker hol_heur_mk = hol_heur_->get_collision_mk();
  if (!hol_heur_mk.points.empty()) {
    collision_mks.markers.emplace_back(hol_heur_mk);
  }
  return collision_mks;
}

visualization_msgs::Marker HolonomicHeuristic::get_collision_mk() {
  visualization_msgs::Marker mk = collision_mk;
  collision_mk.points.clear();
  return mk;
}

// for debug

bool HybirdAstar::update() {
  uint64_t start_time_ms = utils::time_now_ms();

  init_kdtree();

  if (!set_start_and_goal()) return false;
  
  // std::cout << "charge flag: " << get_charge_state() << std::endl;

  if (get_charge_state()) {
    charge_goal_ = current_goal_;
    get_charge_path();
    if (planned_path_->path_points.size() < 2) {
      std::cout << "555" << std::endl;
      return false;
    }

    return true;
  }
  
  hol_heur_->gen_heur_map(start_node_->pose_.head(2), end_node_->pose_.head(2));

  
  directions_ = forward_directions_;
  searching();

  if (end_node_->parent_ == nullptr) {
    if (current_gear_ == plan_interface::Gear::N && open_set_.size() < 50) {
      if (robot_model_->is_reversable()) {
        directions_.insert(directions_.end(), reverse_directions_.begin(), reverse_directions_.end());
      } else if (robot_model_->is_rotatable()) {
        directions_.insert(directions_.end(), rotate_directions_.begin(), rotate_directions_.end());
      }
      std::cout << "directions size: " << directions_.size() << std::endl;
      searching();
    }

    if (end_node_->parent_ == nullptr) {
      std::cout << "no path found" << std::endl;
      return false;
    }
  }

  std::cout << "HYAstar update time: " << utils::time_now_ms() - start_time_ms << " ms" << std::endl;

  set_result();
  if (planned_path_->path_points.size() < 2) {
    return false;
  }
  return true;
}

void HybirdAstar::searching() {
  uint64_t start_ts = utils::time_now_ms();

  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  open_set_.emplace(start_node_->index_, start_node_);
  open_pq_.emplace(start_node_->index_, start_node_->get_cost());

  while (!open_pq_.empty()) {
    if (utils::time_now_ms() - start_ts > cfg_.max_plan_time_ms) {
      std::cout << "searching timeout" << std::endl;
      return;
    }
    int current_idx = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node> current = open_set_[current_idx];
    if (analytic_expansion(current)) {
      break;
    }
    close_set_.emplace(current->index_, current);

    std::vector<std::shared_ptr<Node>> neighbors = gen_neighbors(current);

    for (auto & neighbor : neighbors) {
      if (close_set_.find(neighbor->index_) != close_set_.end()) { continue; }
      if (is_collision(neighbor->pose_)) { close_set_.emplace(neighbor->index_, neighbor); continue; }
      if (is_collision(neighbor->path_)) { continue; }
      for (int i = 0; i < neighbor->path_.size(); ++i) {
        geometry_msgs::Point p;
        p.x = neighbor->path_[i][0];
        p.y = neighbor->path_[i][1];
        expand_path_mk.points.push_back(p);
        if (i != 0 && i != neighbor->path_.size() - 1) {
          expand_path_mk.points.push_back(p);
        }
      }
      calculate_cost(neighbor);
      if (open_set_.find(neighbor->index_) == open_set_.end()) {
        open_set_.emplace(neighbor->index_, neighbor);
        open_pq_.emplace(neighbor->index_, neighbor->get_cost());
      } else {
        if (neighbor->get_cost() < open_set_[neighbor->index_]->get_cost()) {
          open_set_[neighbor->index_] = neighbor;
          open_pq_.emplace(neighbor->index_, neighbor->get_cost());
        }
      }
    }
  }
  std::cout << "searching time: " << utils::time_now_ms() - start_ts << std::endl;
}

bool HybirdAstar::set_start_and_goal() {
  double start_x = current_odom_.pose.pose.position.x;
  double start_y = current_odom_.pose.pose.position.y;
  double start_yaw = utils::geometry::get_yaw_from_ros_q(current_odom_.pose.pose.orientation);
  Eigen::Vector3d start_pose(start_x, start_y, start_yaw);

  double goal_x = current_goal_.pose.position.x;
  double goal_y = current_goal_.pose.position.y;
  double goal_yaw = utils::geometry::get_yaw_from_ros_q(current_goal_.pose.orientation);
  Eigen::Vector3d goal_pose(goal_x, goal_y, goal_yaw);

  double end_x = goal_x - std::cos(goal_yaw) * cfg_.end_point_line_length;
  double end_y = goal_y - std::sin(goal_yaw) * cfg_.end_point_line_length;
  Eigen::Vector3d end_pose(end_x, end_y, goal_yaw);

  if (is_collision(start_pose) || is_collision(goal_pose)) { return false; }

  start_node_ = std::shared_ptr<Node>(new Node(start_pose));
  start_node_->plan_gear_ = current_gear_;
  start_node_->steering_ = 0;

  end_node_ = std::shared_ptr<Node>(new Node(end_pose));

  int end_point_num = std::trunc(cfg_.end_point_line_length / cfg_.arc_step_length);
  if (end_point_num > 1) {
    std::vector<Eigen::Vector3d> end_path;
    for (int i = 1; i < end_point_num; ++i) {
      double x = end_pose[0] + std::cos(goal_yaw) * cfg_.arc_step_length * i;
      double y = end_pose[1] + std::sin(goal_yaw) * cfg_.arc_step_length * i;
      Eigen::Vector3d pose(x, y, goal_yaw);
      end_path.push_back(pose);
    }
    end_path.push_back(goal_pose);
    goal_node_ = std::shared_ptr<Node>(new Node(end_path));
  } else {
    goal_node_ = std::shared_ptr<Node>(new Node(goal_pose));
  }
  goal_node_->parent_ = end_node_;
  goal_node_->plan_gear_ = plan_interface::Gear::D;
  goal_node_->steering_ = 0;

  set_grid();
  start_node_->index_ = grid_->world_pose_to_index(start_pose);
  return true;
}

void HybirdAstar::get_charge_path() {
  std::vector<Eigen::Vector3d> path_nodes;
  path_nodes.push_back(start_node_->pose_);

  double goal_x = current_goal_.pose.position.x;
  double goal_y = current_goal_.pose.position.y;
  double goal_yaw = utils::geometry::get_yaw_from_ros_q(current_goal_.pose.orientation);
  Eigen::Vector3d goal_pose(goal_x, goal_y, goal_yaw);

  double end_x = goal_x + std::cos(goal_yaw) * cfg_.end_point_line_length * 2;
  double end_y = goal_y + std::sin(goal_yaw) * cfg_.end_point_line_length * 2;
  Eigen::Vector3d end_pose(end_x, end_y, goal_yaw);

  int end_point_num = std::trunc(2 * cfg_.end_point_line_length / cfg_.arc_step_length);

  if (end_point_num > 1) {
    for (int i = 1; i < end_point_num; ++i) {
      double x = end_pose[0] - std::cos(goal_yaw) * cfg_.arc_step_length * i;
      double y = end_pose[1] - std::sin(goal_yaw) * cfg_.arc_step_length * i;
      Eigen::Vector3d pose(x, y, goal_yaw);
      path_nodes.push_back(pose);
    }
    path_nodes.push_back(goal_pose); 
  }


  std::shared_ptr<plan_interface::Path> full_path = std::make_shared<plan_interface::Path>();
  for (int i = 0; i < path_nodes.size(); ++i) {
    plan_interface::PathPoint point;
    point.x = path_nodes[i][0];
    point.y = path_nodes[i][1];
    point.yaw = path_nodes[i][2];
    point.gear = plan_interface::Gear::R;
    full_path->path_points.push_back(point);
  }

  planned_path_ = full_path;
}

void HybirdAstar::set_result() {
  std::shared_ptr<Node> current = goal_node_;

  std::vector<std::shared_ptr<Node>> path_nodes;
  while (current->parent_ != nullptr) {
    path_nodes.push_back(current);
    current = current->parent_;
  }
  std::reverse(path_nodes.begin(), path_nodes.end());

  std::shared_ptr<plan_interface::Path> full_path = std::make_shared<plan_interface::Path>();
  for (int i = 0; i < path_nodes.size(); ++i) {
    std::shared_ptr<Node> node = path_nodes[i];
    if (i == 0) {
      plan_interface::PathPoint point;
      point.x = start_node_->pose_[0];
      point.y = start_node_->pose_[1];
      point.yaw = start_node_->pose_[2];
      point.gear = node->plan_gear_;
      full_path->path_points.push_back(point);
    }
    std::vector<Eigen::Vector3d> node_path = node->path_;
    for (auto& pose : node_path) {
      plan_interface::PathPoint point;
      point.x = pose[0];
      point.y = pose[1];
      point.yaw = pose[2];
      point.gear = node->plan_gear_;
      full_path->path_points.push_back(point);
    }
  }

  int gear_switch_index = full_path->path_points.size() - 1;
  plan_interface::Gear first_gear = full_path->path_points[0].gear;
  std::shared_ptr<plan_interface::Path> first_path = std::make_shared<plan_interface::Path>();
  first_path->path_points.push_back(full_path->path_points[0]);
  for (int i = 1; i < full_path->path_points.size(); ++i) {
    if (full_path->path_points[i].gear == first_gear) {
      first_path->path_points.push_back(full_path->path_points[i]);
      gear_switch_index = i;
    } else {
      break;
    }
  }

  if (gear_switch_index < full_path->path_points.size() - 1) {
    first_path->is_last_path = false;
    plan_interface::Gear second_gear = (first_gear == plan_interface::Gear::D) ? plan_interface::Gear::R : plan_interface::Gear::D;
    std::shared_ptr<plan_interface::Path> second_path = std::make_shared<plan_interface::Path>();
    for (int i = gear_switch_index; i < full_path->path_points.size(); ++i) {
      second_path->path_points.push_back(full_path->path_points[i]);
      geometry_msgs::Point p;
      p.x = full_path->path_points[i].x;
      p.y = full_path->path_points[i].y;
      second_path_mk.points.push_back(p);
      if (i == gear_switch_index) {
        second_path->path_points[0].gear = second_gear;
      }
    }
  }
  
  planned_path_ = first_path;
  std::cout << "path_size: " << planned_path_->path_points.size() << std::endl;
}

void HybirdAstar::calculate_cost(const std::shared_ptr<Node>& node) {
  double traj_cost = cfg_.arc_length;
  if (node->plan_gear_ == plan_interface::Gear::R) {
    traj_cost += cfg_.reverse_penalty * cfg_.arc_length;
  }
  if (node->plan_gear_ != node->parent_->plan_gear_) {
    traj_cost += cfg_.gear_switch_penalty * cfg_.arc_length;
  }

  if (node->steering_ != 0) {
    traj_cost += cfg_.steering_penalty * cfg_.arc_length * std::abs(node->steering_)/5.0;
  }
  if (node->steering_ != node->parent_->steering_) {
    traj_cost += cfg_.steering_switch_penalty * cfg_.arc_length * std::abs(node->steering_ - node->parent_->steering_)/10.0;
  }
  node->g_ = traj_cost + node->parent_->g_;

  double heuristic_cost = hol_heur_->get_heur(node->pose_.head<2>());
  node->h_ = heuristic_cost;
  // std::cout << "g: " << node->g_ << "; h: " << node->h_ << std::endl;
}

std::vector<std::shared_ptr<HybirdAstar::Node>> HybirdAstar::gen_neighbors(const std::shared_ptr<Node>& node) {
  std::vector<std::shared_ptr<HybirdAstar::Node>> neighbors;
  for (const auto& direction : directions_) {
    std::vector<Eigen::Vector3d> path = get_arc_traj(node->pose_, direction);
    std::shared_ptr<Node> neighbor = std::shared_ptr<Node>(new Node(path));
    if (direction == "R") {
      neighbor->plan_gear_ = plan_interface::Gear::T;
      neighbor->steering_ = -100;
    } else if (direction == "RRF") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = -10;
    } else if (direction == "RF") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = -5;
    } else if (direction == "RFF") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = -3;
    } else if (direction == "F") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = 0;
    } else if (direction == "LFF") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = 3;
    } else if (direction == "LF") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = 5;
    } else if (direction == "LLF") {
      neighbor->plan_gear_ = plan_interface::Gear::D;
      neighbor->steering_ = 10;
    } else if (direction == "L") {
      neighbor->plan_gear_ = plan_interface::Gear::T;
      neighbor->steering_ = 100;
    } else if (direction == "LLB") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = 10;
    } else if (direction == "LB") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = 5;
    } else if (direction == "LBB") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = 3;
    } else if (direction == "B") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = 0;
    } else if (direction == "RBB") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = -3;
    } else if (direction == "RB") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = -5;
    } else if (direction == "RRB") {
      neighbor->plan_gear_ = plan_interface::Gear::R;
      neighbor->steering_ = -10;
    }
    
    double index = grid_->world_pose_to_index(neighbor->pose_);
    if (index > -1) {
      neighbor->index_ = index;
      neighbor->parent_ = node;
      neighbors.push_back(neighbor);
    }
    
  }
  return neighbors;
}

std::vector<Eigen::Vector3d> HybirdAstar::get_arc_traj(const Eigen::Vector3d& pose_0, const std::string& direction) {
  double kappa = 1/robot_model_->get_min_turn_radius(), s = cfg_.arc_step_length;
  if (direction == "F") {
    kappa = 0.0;
  } else if (direction == "B") {
    kappa = 0.0;
    s = -s;
  } else if (direction == "RF") {
    kappa = -kappa;
  } else if (direction == "RFF") {
    kappa = -kappa/2;
  } else if (direction == "RRF") {
    kappa = -0.3;
  } else if (direction == "LFF") {
    kappa = kappa/2;
  } else if (direction == "LLF") {
    kappa = 0.3;
  } else if (direction == "RB") {
    s = -s;
    kappa = -kappa;
  } else if (direction == "RBB") {
    s = -s;
    kappa = -kappa/2;
  } else if (direction == "LB") {
    s = -s;
  } else if (direction == "LBB") {
    s = -s;
    kappa = kappa/2;
  } else if (direction == "R") {
    kappa = -10*kappa;
  } else if (direction == "L") {
    kappa = 10*kappa;
  }

  Eigen::Vector3d pose = pose_0;
  std::vector<Eigen::Vector3d> path;
  int step_num = std::ceil(cfg_.arc_length / cfg_.arc_step_length);
  for (int i = 0; i < step_num; i++) {
    pose[0] += s * std::cos(pose[2]);
    pose[1] += s * std::sin(pose[2]);
    pose[2] += s * kappa;
    path.push_back(pose);
  }
  return path;
}

bool HybirdAstar::analytic_expansion(const std::shared_ptr<Node>& node) {
  double delta_yaw = utils::geometry::normalize_angle(node->pose_[2] - end_node_->pose_[2]);
  double dirction = std::atan2(end_node_->pose_[0] - node->pose_[0], end_node_->pose_[1] - node->pose_[1]);
  if (std::abs(delta_yaw) > 160/57.3 && std::abs(dirction - node->pose_[2]) < 30/57.3) {
    return false;
  }
  if (utils::geometry::get_distance(node->pose_.head(2), end_node_->pose_.head(2)) < 2.0*cfg_.grid_resolution) {
    return false;
  }
  std::vector<Eigen::Vector3d> path = get_trans_traj(node->pose_, end_node_->pose_);
  if (is_collision(path) || path.empty()) {
    return false;
  }
  end_node_->path_ = path;
  end_node_->parent_ = node;
  end_node_->plan_gear_= plan_interface::Gear::D;

  return true;
}

std::vector<Eigen::Vector3d> HybirdAstar::get_trans_traj(const Eigen::Vector3d& pose_0, const Eigen::Vector3d& pose_f) {
  const double V_0 = 1.0, V_f = 1.0;
  const double max_kappa = 1/robot_model_->get_min_turn_radius();
  double x_0 = pose_0[0];
  double y_0 = pose_0[1];
  double vx_0 = V_0 * std::cos(pose_0[2]);
  double vy_0 = V_0 * std::sin(pose_0[2]);
  double x_f = pose_f[0];
  double y_f = pose_f[1];
  double vx_f = V_f * std::cos(pose_f[2]);
  double vy_f = V_f * std::sin(pose_f[2]);

  double dx = x_f - x_0;
  double dy = y_f - y_0;
  double dvx = vx_f - vx_0;
  double dvy = vy_f - vy_0;

  double a = 36 * (dx * dx + dy * dy);
  double b = -24 * (dx * dvx + dy * dvy + 2 * (dx * vx_0 + dy * vy_0));
  double c = 4 * (dvx * dvx + dvy * dvy + 3 * (vx_0 * vx_0 + vy_0 * vy_0) + 3 * (dvx * vx_0 + dvy * vy_0));

  Eigen::MatrixXd A(4, 4);
  A << 0, 0, 0, a,
      1, 0, 0, b,
      0, 1, 0, c,
      0, 0, 1, 0;
  Eigen::EigenSolver<Eigen::MatrixXd> solver(A);
  Eigen::VectorXcd eigenvalues = solver.eigenvalues();
  double T = std::numeric_limits<double>::max();
  for (int i = 0; i < eigenvalues.size(); ++i) {
    if (eigenvalues[i].imag() == 0) {
      double real = eigenvalues[i].real();
      if (real > 0 && real < T) {
        T = real;
      }
    }
  }
  
  std::vector<Eigen::Vector3d> path;
  // if (T > 3.0) {
  //   return path;
  // }
  double a_1 = 6 * (vx_0 + vx_f) / (T * T) - 12 * (x_f - x_0) / (T * T * T);
  double a_2 = 6 * (vy_0 + vy_f) / (T * T) - 12 * (y_f - y_0) / (T * T * T);
  double b_1 = -2 * (2 * vx_0 + vx_f) / T + 6 * (x_f - x_0) / (T * T);
  double b_2 = -2 * (2 * vy_0 + vy_f) / T + 6 * (y_f - y_0) / (T * T);

  // std::cout << "T = " << T << ", a_1 = " << a_1 << ", b_1 = " << b_1 << ", a_2 = " << a_2 << ", b_2 = " << b_2 << std::endl;
  
  double dt = cfg_.arc_step_length / ((V_0 + V_f) / 2 );
  int n = std::trunc(T / dt);
  if (std::fmod(T/dt, 1.0) > 0.3) {
    n += 1;
  }
  for (int i = 1; i < n; i++) {
    double t = i * dt;
    double x = x_0 + vx_0 * t + 0.5 * b_1 * t * t + a_1 * t * t * t / 6;
    double y = y_0 + vy_0 * t + 0.5 * b_2 * t * t + a_2 * t * t * t / 6;
    double vx = vx_0 + b_1 * t + 0.5 * a_1 * t * t;
    double vy = vy_0 + b_2 * t + 0.5 * a_2 * t * t;
    double theta = std::atan2(vy, vx);
    path.emplace_back(Eigen::Vector3d(x, y, theta));
    double ax = a_1 * t + b_1;
    double ay = a_2 * t + b_2;

    double kappa = (ax * vy - ay * vx) / std::pow(vx*vx + vy*vy, 1.5);
    // std::cout<< "kappa " << i << " = " << kappa << std::endl;
    if (std::abs(kappa) > max_kappa) {
      path.clear();
      return path;
    }
  }
  path.emplace_back(pose_f);
  return path;
}

bool HybirdAstar::is_collision(const std::vector<Eigen::Vector3d>& path) {
  if (path.empty()) { return false; }
  for (const auto& pose : path) {
    if (is_collision(pose)) {
      return true;
    }
  }
  return false;
}

bool HybirdAstar::is_collision(const Eigen::Vector3d& pose) {
  Eigen::Vector2d near = obst_kdtree_->get_near(pose.head<2>());
  std::vector<Eigen::Vector2d> bounding_polygon;
  if(get_charge_state() || current_goal_ == charge_goal_)
    std::vector<Eigen::Vector2d> bounding_polygon = robot_model_->get_bounding_rectangle(pose, cfg_.charge_search_safe_buffer, cfg_.charge_search_safe_buffer, cfg_.charge_search_safe_buffer);
  else 
    std::vector<Eigen::Vector2d> bounding_polygon = robot_model_->get_bounding_rectangle(pose, cfg_.search_safe_buffer, cfg_.search_safe_buffer, cfg_.search_safe_buffer);

  if (robot_model_->is_point_in_polygon(near, bounding_polygon)) {
    return true;
  }
  return false;
}



HolonomicHeuristic::HolonomicHeuristic(std::shared_ptr<RobotModel> robot_model, std::shared_ptr<GridMap> grid, 
                                      std::shared_ptr<utils::kdtree::KDTree2D> obst_kdtree, double safe_buffer)
 : robot_model_(robot_model), grid_(grid), obst_kdtree_(obst_kdtree), safe_buffer_(safe_buffer) {
  init_mk();
 }


void HolonomicHeuristic::gen_heur_map(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& goal_pos) {
  uint64_t start_ts = utils::time_now_ms();
  std::unordered_map<int, std::shared_ptr<Node>> open_set;
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_pq;
  
  heuristic_map_.clear();
  Eigen::Vector2i goal_coord = grid_->world_to_grid(goal_pos);
  std::shared_ptr<Node> goal_node = std::make_shared<Node>(goal_coord);
  goal_node->index_ = grid_->grid_to_index(goal_coord);

  open_set.emplace(goal_node->index_, goal_node);
  open_pq.emplace(goal_node);

  while (!open_pq.empty()) {
    auto node = open_pq.top();
    open_pq.pop();
    // open_set.erase(node->get_index());
    heuristic_map_.emplace(node->index_, node);
    for (auto neighbor : get_neighbors(node)) {
      if (heuristic_map_.find(neighbor->index_) != heuristic_map_.end()) { continue; }
      if (!is_valid(neighbor)) { continue; }
      if (open_set.find(neighbor->index_) == open_set.end()) {
        neighbor->parent_ = node;
        open_set.emplace(neighbor->index_, neighbor);
        open_pq.emplace(neighbor);
      } else {
        if (open_set[neighbor->index_]->g_ > neighbor->g_) {
          open_set[neighbor->index_]->parent_ = node;
          open_set[neighbor->index_]->g_ = neighbor->g_;
          open_pq.emplace(neighbor);
        }
      }
    }
  }
  std::cout << "heuristic time: " << utils::time_now_ms() - start_ts << std::endl;
}

double HolonomicHeuristic::get_heur(const Eigen::Vector2d& pos) {
  int index = grid_->world_posi_to_index(pos);
  if (heuristic_map_.find(index) != heuristic_map_.end()) {
    return heuristic_map_[index]->g_ * grid_->get_resolution();
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

std::vector<std::shared_ptr<HolonomicHeuristic::Node>> HolonomicHeuristic::get_neighbors(const std::shared_ptr<Node>& node) {
  std::vector<std::shared_ptr<Node>> neighbors;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (i == 0 && j == 0) continue;
      auto neighbor = std::make_shared<Node>(Eigen::Vector2i(node->coord_.x() + i, node->coord_.y() + j));
      if (i!=0 && j!=0) {
        neighbor->g_ = node->g_ + 1.414;
      } else {
        neighbor->g_ = node->g_ + 1;
      }
      neighbor->index_ = grid_->grid_to_index(neighbor->coord_);
      neighbors.push_back(neighbor);
    }
  }
  return neighbors;
}

bool HolonomicHeuristic::is_valid(const std::shared_ptr<Node>& node) {
  Eigen::Vector2d pos = grid_->grid_to_world(node->coord_);

  // if (!grid_->in_grid(node->coord_)) {
  if (!grid_->in_range(node->coord_)) {
    geometry_msgs::Point p;
    p.x = pos.x();
    p.y = pos.y();
    collision_mk.points.push_back(p);
    return false;
  }
  if (is_collision(pos)) {
    return false;
  }
  return true;
}

bool HolonomicHeuristic::is_collision(const std::shared_ptr<Node>& node) {
  Eigen::Vector2d pos = grid_->grid_to_world(node->coord_);
  return is_collision(pos);
}

bool HolonomicHeuristic::is_collision(const Eigen::Vector2d& pos) {
  if (obst_kdtree_->get_dist(pos) < robot_model_->get_width()/2 + safe_buffer_) {
    geometry_msgs::Point p;
    p.x = pos.x();
    p.y = pos.y();
    collision_mk.points.push_back(p);
    return true;
  }
  return false;
}

void HybirdAstar::set_grid() {
  Eigen::Vector2d start_pos = start_node_->pose_.head(2);
  Eigen::Vector2d goal_pos = end_node_->pose_.head(2);
  Eigen::Vector2d center = (start_pos + goal_pos) / 2;
  double length = (start_pos - goal_pos).norm();
  double range = length/2 + cfg_.grid_radius_offset;

  GridMeta meta;
  meta.resolution = cfg_.grid_resolution;
  meta.width = meta.height = static_cast<int>(2 * range / cfg_.grid_resolution);
  meta.origin_x = center.x() - range;
  meta.origin_y = center.y() - range;
  meta.rotation = 0.0;
  grid_->set_meta(meta);
  grid_->set_angle_resolution(cfg_.angle_resolution);
}

} // namespace hybird_astar

} // namespace global_planner