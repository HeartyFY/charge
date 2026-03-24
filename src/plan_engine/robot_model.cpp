#include <fstream>
#include <iostream>
#include "robot_model.h"
#include "geometry.h"


RobotModel::RobotModel(const std::string &robot_params_file) {
  from_file(robot_params_file);
}

void RobotModel::from_file(const std::string &robot_params_file) {
  std::ifstream json_stream(robot_params_file);
  if (!json_stream.is_open()) {
    throw std::runtime_error("Unable to open robot_params_file: " + robot_params_file);
  }
  nlohmann::json json = nlohmann::json::parse(json_stream);
  from_json(json);
  json_stream.close();
}

void RobotModel::from_json(const nlohmann::json &robot_params_json) {
  try {
    robot_name_ = robot_params_json.at("robot_name").get<std::string>();
    robot_type_ = robot_params_json.at("robot_type").get<std::string>();
    nlohmann::json geom_params = robot_params_json.at("geom_params");
    params_.geom.length = geom_params.at("length").get<double>();
    params_.geom.width = geom_params.at("width").get<double>();
    params_.geom.height = geom_params.at("height").get<double>();
    params_.geom.rear_overhang = geom_params.at("rear_overhang").get<double>();

    nlohmann::json dyn_params = robot_params_json.at("dyn_params");
    params_.dyn.max_speed = dyn_params.at("max_speed").get<double>();
    params_.dyn.max_accel = dyn_params.at("max_accel").get<double>();
    params_.dyn.max_decel = dyn_params.at("max_decel").get<double>();
    params_.dyn.emergency_decel = dyn_params.at("emergency_decel").get<double>();
    params_.dyn.max_yaw_rate = dyn_params.at("max_yaw_rate").get<double>();
    params_.dyn.max_yaw_accel = dyn_params.at("max_yaw_accel").get<double>();

    params_.dyn.brake_distance = dyn_params.contains("brake_distance")?  dyn_params.at("brake_distance").get<double>() : 0.2;
    params_.dyn.min_safe_distance = dyn_params.contains("min_safe_distance")?  dyn_params.at("min_safe_distance").get<double>() : 0.2;
    params_.dyn.min_turn_radius = dyn_params.contains("min_turn_radius")?  dyn_params.at("min_turn_radius").get<double>() : 0.5;
    
    params_.dyn.static_max_speed = dyn_params.contains("static_max_speed")?  dyn_params.at("static_max_speed").get<double>() : 0.05;
    params_.dyn.static_max_yaw_rate = dyn_params.contains("static_max_yaw_rate")?  dyn_params.at("static_max_yaw_rate").get<double>()/57.3 : 1/57.3;
    params_.dyn.static_min_time = dyn_params.contains("static_min_time")?  dyn_params.at("static_min_time").get<int>() : 1000;

    
    params_.dyn.reversable = dyn_params.contains("reversable")?  dyn_params.at("reversable").get<bool>() : false;
    params_.dyn.rotatable = dyn_params.contains("rotatable")?  dyn_params.at("rotatable").get<bool>() : false;

    nlohmann::json bound_params = robot_params_json.at("bound_params");
    params_.bound.front_expansion = bound_params.at("front_expansion").get<double>();
    params_.bound.rear_expansion = bound_params.at("rear_expansion").get<double>();
    params_.bound.side_expansion = bound_params.at("side_expansion").get<double>();
    params_.bound.loc_map_valid_range = bound_params.at("loc_map_valid_range").get<double>();
    params_.bound.loc_map_x_offset = bound_params.at("loc_map_x_offset").get<double>();

    std::cout << "Robot params loaded successfully" << std::endl;
    std::cout << "Robot name: " << robot_name_ << std::endl;
    std::cout << "Robot type: " << robot_type_ << std::endl;
    std::cout << "Length: " << params_.geom.length << std::endl;
    std::cout << "Width: " << params_.geom.width << std::endl;
    std::cout << "Height: " << params_.geom.height << std::endl;
    std::cout << "Rear overhang: " << params_.geom.rear_overhang << std::endl;
    std::cout << "Max speed: " << params_.dyn.max_speed << std::endl;
    std::cout << "Max accel: " << params_.dyn.max_accel << std::endl;
    std::cout << "Max decel: " << params_.dyn.max_decel << std::endl;
    std::cout << "Emergency decel: " << params_.dyn.emergency_decel << std::endl;
    std::cout << "Max yaw rate: " << params_.dyn.max_yaw_rate << std::endl;
    std::cout << "Max yaw accel: " << params_.dyn.max_yaw_accel << std::endl;
    std::cout << "brake_distance: " << params_.dyn.brake_distance << std::endl;
    std::cout << "min_safe_distance: " << params_.dyn.min_safe_distance << std::endl;
    std::cout << "min_turn_radius: " << params_.dyn.min_turn_radius << std::endl;
    std::cout << "static_max_speed: " << params_.dyn.static_max_speed << std::endl;
    std::cout << "static_max_yaw_rate: " << params_.dyn.static_max_yaw_rate << std::endl;
    std::cout << "static_min_time: " << params_.dyn.static_min_time << std::endl;
    std::cout << "reversable: " << params_.dyn.reversable << std::endl;
    std::cout << "rotatable: " << params_.dyn.rotatable << std::endl;
    std::cout << "Front expansion: " << params_.bound.front_expansion << std::endl;
    std::cout << "Rear expansion: " << params_.bound.rear_expansion << std::endl;
    std::cout << "Side expansion: " << params_.bound.side_expansion << std::endl;
    std::cout << "Loc map valid range: " << params_.bound.loc_map_valid_range << std::endl;
    std::cout << "Loc map x offset: " << params_.bound.loc_map_x_offset << std::endl;
    

  } catch (nlohmann::json::exception &e) {
    throw std::runtime_error("Failed to load robot parameters: " + std::string(e.what()));
  }
  
}

std::vector<Eigen::Vector2d> RobotModel::get_ego_rectangle(double x, double y, double yaw) {
  double front = params_.geom.length - params_.geom.rear_overhang;
  double rear = params_.geom.rear_overhang;
  double side = params_.geom.width/2;

  Eigen::Vector2d lf, rf, rr, lr;
  lf << front, side;
  rf << front, -side;
  rr << -rear, -side;
  lr << -rear, side;

  Eigen::Vector2d translation(x, y);
  Eigen::Rotation2Dd rotation(yaw); 

  std::vector<Eigen::Vector2d> rectangle;
  rectangle.push_back(rotation * lf + translation);
  rectangle.push_back(rotation * rf + translation);
  rectangle.push_back(rotation * rr + translation);
  rectangle.push_back(rotation * lr + translation);

  return rectangle;
}

std::vector<Eigen::Vector2d> RobotModel::get_bounding_rectangle(double x, double y, double yaw) {
  double front = params_.geom.length - params_.geom.rear_overhang + params_.bound.front_expansion;
  double rear = params_.geom.rear_overhang + params_.bound.rear_expansion;
  double side = params_.geom.width/2 + params_.bound.side_expansion;
  
  Eigen::Vector2d lf, rf, rr, lr;
  lf << front, side;
  rf << front, -side;
  rr << -rear, -side;
  lr << -rear, side;

  Eigen::Vector2d translation(x, y);
  Eigen::Rotation2Dd rotation(yaw); 

  std::vector<Eigen::Vector2d> rectangle;
  rectangle.push_back(rotation * lf + translation);
  rectangle.push_back(rotation * rf + translation);
  rectangle.push_back(rotation * rr + translation);
  rectangle.push_back(rotation * lr + translation);

  return rectangle;
}

std::vector<Eigen::Vector2d> RobotModel::get_bounding_rectangle(Eigen::Vector3d pose, double front_expansion, double rear_expansion, double side_expansion) {
  double front = params_.geom.length - params_.geom.rear_overhang + front_expansion;
  double rear = params_.geom.rear_overhang + rear_expansion;
  double side = params_.geom.width/2 + side_expansion;
  
  Eigen::Vector2d lf, rf, rr, lr;
  lf << front, side;
  rf << front, -side;
  rr << -rear, -side;
  lr << -rear, side;

  Eigen::Vector2d translation(pose[0], pose[1]);
  Eigen::Rotation2Dd rotation(pose[2]); 

  std::vector<Eigen::Vector2d> rectangle;
  rectangle.push_back(rotation * lf + translation);
  rectangle.push_back(rotation * rf + translation);
  rectangle.push_back(rotation * rr + translation);
  rectangle.push_back(rotation * lr + translation);

  return rectangle;
}


Eigen::Vector3d RobotModel::kinematic_diff(const Eigen::Vector3d& state, double v, double w_input) {
  Eigen::Vector3d derivative;
  derivative[0] = v * cos(state[2]);
  derivative[1] = v * sin(state[2]);
  derivative[2] = w_input;
  return derivative;
}

Eigen::Vector3d RobotModel::rk4_diff(const Eigen::Vector3d& state, double v, double w_input, double dt) {
  Eigen::Vector3d k1 = kinematic_diff(state, v, w_input);
  Eigen::Vector3d k2 = kinematic_diff({state[0] + k1[0] * dt / 2, state[1] + k1[1] * dt / 2, state[2] + k1[2] * dt / 2}, v, w_input);
  Eigen::Vector3d k3 = kinematic_diff({state[0] + k2[0] * dt / 2, state[1] + k2[1] * dt / 2, state[2] + k2[2] * dt / 2}, v, w_input);
  Eigen::Vector3d k4 = kinematic_diff({state[0] + k3[0] * dt, state[1] + k3[1] * dt, state[2] + k3[2] * dt}, v, w_input);

  Eigen::Vector3d new_state;
  new_state[0] = state[0] + (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) * dt / 6;
  new_state[1] = state[1] + (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) * dt / 6;
  new_state[2] = state[2] + (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) * dt / 6;

  return new_state;
}

std::set<std::pair<int, int>> RobotModel::get_covered_cells(
  const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector3d& map_origin, double resolution) {
  std::set<std::pair<int, int>> covered_cells;

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& vertex : polygon) {
      min_x = std::min(min_x, vertex.x());
      max_x = std::max(max_x, vertex.x());
      min_y = std::min(min_y, vertex.y());
      max_y = std::max(max_y, vertex.y());
  }

  int min_cell_x = static_cast<int>(std::floor((min_x - map_origin.x()) / resolution));
  int max_cell_x = static_cast<int>(std::ceil((max_x - map_origin.x()) / resolution));
  int min_cell_y = static_cast<int>(std::floor((min_y - map_origin.y()) / resolution));
  int max_cell_y = static_cast<int>(std::ceil((max_y - map_origin.y()) / resolution));

  for (int cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
      for (int cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
          double wx = cell_x * resolution + map_origin.x();
          double wy = cell_y * resolution + map_origin.y();
          if (is_point_in_polygon(Eigen::Vector2d(wx, wy), polygon)) {
              covered_cells.emplace(cell_x, cell_y);
          }
      }
  }

  return covered_cells;
}

bool RobotModel::is_point_in_polygon(const Eigen::Vector2d& point,
                                     const std::vector<Eigen::Vector2d>& polygon) {
  int intersections = 0;
  size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
      const auto& p1 = polygon[i];
      const auto& p2 = polygon[(i + 1) % n];

      if ((point.y() > std::min(p1.y(), p2.y())) &&
          (point.y() <= std::max(p1.y(), p2.y())) &&
          (point.x() <= std::max(p1.x(), p2.x()))) {
          double x_intersect = p1.x() + (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y());
          if (x_intersect > point.x()) {
              intersections++;
          }
      }
  }

  return (intersections % 2) == 1;
}

std::set<std::pair<int, int>> RobotModel::get_covered_loc_cells(const std::vector<Eigen::Vector2d>& polygon) {
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::lowest();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::lowest();

  for (const auto& vertex : polygon) {
    auto [lx, ly] = world_to_loc_map_coords(vertex.x(), vertex.y());
    min_x = std::min(min_x, lx);
    max_x = std::max(max_x, lx);
    min_y = std::min(min_y, ly);
    max_y = std::max(max_y, ly);
  }

  std::set<std::pair<int, int>> covered_cells;

  for (int cell_x = min_x; cell_x <= max_x; ++cell_x) {
    for (int cell_y = min_y; cell_y <= max_y; ++cell_y) {
      auto [wx, wy] = loc_map_to_world_coords(cell_x, cell_y);
      if (is_point_in_polygon(Eigen::Vector2d(wx, wy), polygon)) {
        covered_cells.emplace(cell_x, cell_y);
      }
    }
  }

  return covered_cells;
}

std::set<std::pair<int, int>> RobotModel::get_covered_glob_cells(const std::vector<Eigen::Vector2d>& polygon) {
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::lowest();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::lowest();

  for (const auto& vertex : polygon) {
    auto [gx, gy] = world_to_glob_map_coords(vertex.x(), vertex.y());
    min_x = std::min(min_x, gx);
    max_x = std::max(max_x, gx);
    min_y = std::min(min_y, gy);
    max_y = std::max(max_y, gy);
  }

  std::set<std::pair<int, int>> covered_cells;

  for (int cell_x = min_x; cell_x <= max_x; ++cell_x) {
    for (int cell_y = min_y; cell_y <= max_y; ++cell_y) {
      auto [wx, wy] = glob_map_to_world_coords(cell_x, cell_y);
      if (is_point_in_polygon(Eigen::Vector2d(wx, wy), polygon)) {
        covered_cells.emplace(cell_x, cell_y);
      }
    } 
  }

  return covered_cells;
}

bool RobotModel::is_within_glob_map_bounds(int gx, int gy) {
  return gx >= 0 && gx < glob_map_meta_.width && gy >= 0 && gy < glob_map_meta_.height;
}

bool RobotModel::is_within_loc_map_bounds(int lx, int ly) {
  return lx >= 0 && lx < loc_map_meta_.width && ly >= 0 && ly < loc_map_meta_.height;
}

bool RobotModel::is_within_valid_loc_map_bounds(int lx, int ly) {
  int width = loc_map_meta_.width;
  int height = loc_map_meta_.height;
  double resolution = loc_map_meta_.resolution;
  double range = params_.bound.loc_map_valid_range / resolution;
  double x_offset = params_.bound.loc_map_x_offset / resolution;
  return lx >= (width/2+x_offset) && lx < (width/2+range) && ly >= (height/2-range) && ly < (height/2+range);
}

std::pair<int, int> RobotModel::world_to_glob_map_coords(double wx, double wy) {
  double x_map = (wx - glob_map_meta_.origin_x) * std::cos(-glob_map_meta_.rotation) - (wy - glob_map_meta_.origin_y) * std::sin(-glob_map_meta_.rotation);
  double y_map = (wx - glob_map_meta_.origin_x) * std::sin(-glob_map_meta_.rotation) + (wy - glob_map_meta_.origin_y) * std::cos(-glob_map_meta_.rotation);

  int gx = static_cast<int>(x_map / glob_map_meta_.resolution);
  int gy = static_cast<int>(y_map / glob_map_meta_.resolution);

  return {gx, gy};
}

std::pair<int, int> RobotModel::world_to_loc_map_coords(double wx, double wy) {
  double x_map = (wx - loc_map_meta_.origin_x) * std::cos(-loc_map_meta_.rotation) - (wy - loc_map_meta_.origin_y) * std::sin(-loc_map_meta_.rotation);
  double y_map = (wx - loc_map_meta_.origin_x) * std::sin(-loc_map_meta_.rotation) + (wy - loc_map_meta_.origin_y) * std::cos(-loc_map_meta_.rotation);

  int lx = static_cast<int>(x_map / loc_map_meta_.resolution);
  int ly = static_cast<int>(y_map / loc_map_meta_.resolution);

  return {lx, ly};
}

std::pair<double, double> RobotModel::glob_map_to_world_coords(int gx, int gy) {
  double x_map = gx * glob_map_meta_.resolution;
  double y_map = gy * glob_map_meta_.resolution;

  double w_x = x_map * std::cos(glob_map_meta_.rotation) - y_map * std::sin(glob_map_meta_.rotation) + glob_map_meta_.origin_x;
  double w_y = x_map * std::sin(glob_map_meta_.rotation) + y_map * std::cos(glob_map_meta_.rotation) + glob_map_meta_.origin_y;

  return {w_x, w_y};
}

std::pair<double, double> RobotModel::loc_map_to_world_coords(int lx, int ly) {
  double x_map = lx * loc_map_meta_.resolution;
  double y_map = ly * loc_map_meta_.resolution;

  double w_x = x_map * std::cos(loc_map_meta_.rotation) - y_map * std::sin(loc_map_meta_.rotation) + loc_map_meta_.origin_x;
  double w_y = x_map * std::sin(loc_map_meta_.rotation) + y_map * std::cos(loc_map_meta_.rotation) + loc_map_meta_.origin_y;

  return {w_x, w_y};
}

int RobotModel::get_pos_index_in_glob_map(double wx, double wy) {
  auto [gx, gy] = world_to_glob_map_coords(wx, wy);
  return get_map_coord_index_in_glob_map(gx, gy);
}
int RobotModel::get_map_coord_index_in_glob_map(int gx, int gy) {
  if (!is_within_glob_map_bounds(gx, gy)) {
    return -1;
  }
  return gx + gy * glob_map_meta_.width;
}
int RobotModel::get_pos_index_in_loc_map(double wx, double wy) {
  auto [lx, ly] = world_to_loc_map_coords(wx, wy);
  return get_map_coord_index_in_loc_map(lx, ly);
}
int RobotModel::get_map_coord_index_in_loc_map(int lx, int ly) {
  if (!is_within_loc_map_bounds(lx, ly)) {
    return -1;
  }
  return lx + ly * loc_map_meta_.width;
}

int RobotModel::get_pose_index_in_glob_map(double wx, double wy, double yaw) {
  auto [gx, gy] = world_to_glob_map_coords(wx, wy);
  return get_map_coord_index_in_glob_map(gx, gy, yaw);
}
int RobotModel::get_map_coord_index_in_glob_map(int gx, int gy, double yaw) {
  if (!is_within_glob_map_bounds(gx, gy)) {
    return -1;
  }
  yaw = utils::geometry::normalize_angle(yaw);
  return gx + gy * glob_map_meta_.width + static_cast<int>(yaw / 0.1 * glob_map_meta_.width * glob_map_meta_.height);
}

void RobotModel::reset_state() {
  state_.remain_s = 0.0;
  state_.lat_error = 0.0;
  state_.heading_error = 0.0;
  state_.standstill = true;
  state_.gear = plan_interface::Gear::N;
  // charge_state_ = false;
}