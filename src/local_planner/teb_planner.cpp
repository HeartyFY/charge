#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <memory>

#include "geometry.h"
#include "teb_planner.h"

namespace local_planner {

TEBPlanner::TEBPlanner(std::shared_ptr<RobotModel> robot_model)
  : robot_model_(robot_model), planned_trajectory_(std::make_shared<plan_interface::Trajectory>()) {}

void TEBPlanner::feed_data(const MapInfoMsg &map, const OdomInfoMsg &odom,
                          const ObstacleMsg &obs, const plan_interface::Path &glob_path) {
  current_map_ = map;
  current_odom_ = odom;
  current_obs_ = obs;
  glob_path_ = glob_path;
}

std::shared_ptr<plan_interface::Trajectory> TEBPlanner::get_result() {
  return planned_trajectory_;
}

bool TEBPlanner::update() {
  if (glob_path_.path_points.empty()) {
    std::cerr << "Global path is empty. Cannot plan trajectory.\n";
    return false;
  }

  double cur_x = current_odom_.pose.pose.position.x;
  double cur_y = current_odom_.pose.pose.position.y;
  double cur_yaw = utils::geometry::get_yaw_from_ros_q(current_odom_.pose.pose.orientation);
  int start_index = utils::geometry::get_closest_path_point_index(Eigen::Vector2d(cur_x, cur_y), glob_path_);
  start_index = std::max(start_index - 10, 0);
  int end_index = std::min(static_cast<size_t>(start_index + 60), glob_path_.path_points.size());
  std::shared_ptr<plan_interface::Trajectory> this_traj = std::make_shared<plan_interface::Trajectory>();

  plan_interface::TrajectoryPoint start_point;
  start_point.x = glob_path_.path_points[start_index].x;
  start_point.y = glob_path_.path_points[start_index].y;
  start_point.yaw = glob_path_.path_points[start_index].yaw;
  // start_point.yaw = cur_yaw;
  start_point.v = 0.0;
  start_point.yaw_rate = 0.0;
  start_point.relative_time = 0.0;
  this_traj->trajectory_points.push_back(start_point);

  for (size_t i = start_index + 1 ; i < end_index; ++i) {
    const auto &path_point = glob_path_.path_points[i];
    plan_interface::TrajectoryPoint traj_point;
    traj_point.x = path_point.x;
    traj_point.y = path_point.y;
    traj_point.yaw = path_point.yaw;
    traj_point.v = 0.0;
    traj_point.yaw_rate = 0.0;
    double distance = std::hypot(traj_point.x - this_traj->trajectory_points.back().x, traj_point.y - this_traj->trajectory_points.back().y);
    traj_point.relative_time = this_traj->trajectory_points.back().relative_time + 2*distance / robot_model_->get_max_speed();                                    
    this_traj->trajectory_points.push_back(traj_point);
  }

  const int max_iterations = 100;
  const double tolerance = 1e-4;
  double learning_rate = 0.1; // Initial learning rate
  double decay_rate = 0.99;   // Learning rate decay

  for (int iter = 0; iter < max_iterations; ++iter) {
    double cost = optimize(this_traj, learning_rate);
    if (cost < tolerance) {
      std::cout << "Trajectory optimization converged after " << iter << " iterations.\n";
      break;
    }
    learning_rate *= decay_rate; // Decay learning rate
  }


  construct_traj(this_traj);
  return true;
}

double TEBPlanner::optimize(std::shared_ptr<plan_interface::Trajectory> init_traj, double learning_rate) {
  double total_cost = 0.0;

  // Weights for cost terms
  const double w_smoothness = 1.0;
  const double w_yaw_alignment = 1.0;
  const double w_velocity = 0.1;
  const double w_acceleration = 0.1;
  const double w_time = 0.5;
  const double w_obstacle = 1e3;
  const double w_tangent = 1.0; // Weight for tangent alignment

  for (size_t i = 0; i < init_traj->trajectory_points.size(); ++i) {
    auto &curr = init_traj->trajectory_points[i];

    // Tangent alignment cost for start and end points
    if (i == 0 || i == init_traj->trajectory_points.size() - 1) {
      double tangent_yaw = compute_tangent_yaw(init_traj, i);
      double tangent_cost = w_tangent * std::pow(utils::geometry::normalize_angle(curr.yaw - tangent_yaw), 2);
      total_cost += tangent_cost;
    }

    if (i == 0 || i == init_traj->trajectory_points.size() - 1) {
      continue; // Skip fixed start and end points for other costs
    }

    auto &prev = init_traj->trajectory_points[i - 1];
    auto &next = init_traj->trajectory_points[i + 1];

    // 1. Smoothness cost (minimize curvature changes)
    double dx1 = curr.x - prev.x;
    double dy1 = curr.y - prev.y;
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;

    double angle1 = std::atan2(dy1, dx1);
    double angle2 = std::atan2(dy2, dx2);
    double angle_diff = utils::geometry::normalize_angle(angle2 - angle1);
    double smoothness_cost = w_smoothness * std::pow(angle_diff, 2);

    // 2. Yaw alignment cost (ensure yaw matches tangent direction)
    double yaw_alignment_cost = w_yaw_alignment * std::pow(utils::geometry::normalize_angle(curr.yaw - angle1), 2);

    // 3. Velocity and acceleration constraints
    double dt = curr.relative_time - prev.relative_time;
    double v_x = (curr.x - prev.x) / dt;
    double v_y = (curr.y - prev.y) / dt;
    double linear_speed = std::sqrt(v_x * v_x + v_y * v_y);
    double angular_speed = (curr.yaw - prev.yaw) / dt;

    double linear_acc = (linear_speed - std::sqrt(dx1 * dx1 + dy1 * dy1) / dt) / dt;
    double angular_acc = (angular_speed - (curr.yaw - prev.yaw) / dt) / dt;

    double velocity_cost = w_velocity * (std::pow(std::max(0.0, linear_speed - robot_model_->get_max_speed()), 2) +
                                         std::pow(std::max(0.0, std::fabs(angular_speed) - robot_model_->get_max_yaw_rate()), 2));

    double acceleration_cost = w_acceleration * (std::pow(std::max(0.0, std::fabs(linear_acc) - robot_model_->get_max_accel()), 2) +
                                                 std::pow(std::max(0.0, std::fabs(angular_acc) - robot_model_->get_max_yaw_accel()), 2));

    // 4. Time cost (minimize total time)
    double time_cost = w_time * dt;

    // 5. Obstacle cost (ensure points do not overlap with obstacles)
    // double obstacle_cost = is_collision(curr.x, curr.y, curr.yaw) ? w_obstacle : 0.0;
    double obstacle_cost = 0.0;

    // Total cost
    total_cost += smoothness_cost + yaw_alignment_cost + velocity_cost + acceleration_cost + time_cost + obstacle_cost;

    // Gradient descent update
    double grad_x = 2 * (curr.x - prev.x) + 2 * (curr.x - next.x) + (obstacle_cost > 0 ? 1e6 : 0);
    double grad_y = 2 * (curr.y - prev.y) + 2 * (curr.y - next.y) + (obstacle_cost > 0 ? 1e6 : 0);
    double grad_yaw = 2 * (curr.yaw - angle1);

    curr.x -= learning_rate * grad_x;
    curr.y -= learning_rate * grad_y;
    curr.yaw -= learning_rate * grad_yaw;
    curr.yaw = utils::geometry::normalize_angle(curr.yaw);

    // Enforce velocity and acceleration limits
    if (linear_speed > robot_model_->get_max_speed()) linear_speed = robot_model_->get_max_speed();
    if (std::fabs(angular_speed) > robot_model_->get_max_yaw_rate()) angular_speed = robot_model_->get_max_yaw_rate();
  }

  return total_cost;
}

double TEBPlanner::compute_tangent_yaw(std::shared_ptr<plan_interface::Trajectory> traj, size_t index) {
  if (index == 0) {
    // Tangent at start point: direction from start to second point
    const auto &start = traj->trajectory_points[0];
    const auto &next = traj->trajectory_points[1];
    return std::atan2(next.y - start.y, next.x - start.x);
  } else if (index == traj->trajectory_points.size() - 1) {
    // Tangent at end point: direction from second last to end point
    const auto &prev = traj->trajectory_points[traj->trajectory_points.size() - 2];
    const auto &end = traj->trajectory_points[traj->trajectory_points.size() - 1];
    return std::atan2(end.y - prev.y, end.x - prev.x);
  } else {
    // Tangent at intermediate points: average direction from previous to next point
    const auto &prev = traj->trajectory_points[index - 1];
    const auto &next = traj->trajectory_points[index + 1];
    return std::atan2(next.y - prev.y, next.x - prev.x);
  }
}


void TEBPlanner::construct_traj(std::shared_ptr<plan_interface::Trajectory> opt_traj) {
  opt_traj->length = 0.0;
  for (size_t i = 1; i < opt_traj->trajectory_points.size(); ++i) {
    const auto &prev = opt_traj->trajectory_points[i - 1];
    const auto &curr = opt_traj->trajectory_points[i];
    double distance = std::hypot(curr.x - prev.x, curr.y - prev.y);
    opt_traj->length += distance;
  }
  opt_traj->segments = opt_traj->trajectory_points.size() - 1;
  opt_traj->total_time = opt_traj->trajectory_points.back().relative_time;

  planned_trajectory_ = opt_traj;
}

bool TEBPlanner::is_collision(double x, double y, double yaw) {
  int width = current_map_.info.width;
  int height = current_map_.info.height;
  double resolution = current_map_.info.resolution;
  Eigen::Vector2d map_origin(current_map_.info.origin.position.x, current_map_.info.origin.position.y);

  std::vector<Eigen::Vector2d> bounding_box = robot_model_->get_bounding_rectangle(x, y, yaw);
  std::set<std::pair<int, int>> covered_cells = utils::geometry::get_covered_cells_in_map(bounding_box, map_origin, resolution);

  for (const auto& cell : covered_cells) {
    int nx = cell.first;
    int ny = cell.second;
    if (current_map_.data[ny * width + nx] > 0) {
      return true;
    }
  }
  return false;
}

} // namespace local_planner
