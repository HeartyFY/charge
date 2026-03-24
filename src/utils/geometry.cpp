#include "geometry.h"
#include <stdexcept>

namespace utils {
namespace geometry {

double clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

double normalize_angle(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

double get_yaw_from_ros_q(const geometry_msgs::Quaternion &q) {
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::Quaternion get_ros_q_from_yaw(const double &yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2);
  q.w = std::cos(yaw / 2);
  return q;
}

Eigen::Vector3d transform_pose_2b(Eigen::Vector3d pose_w, Eigen::Vector3d ego_pose_w) {
  double ego_x = ego_pose_w[0];
  double ego_y = ego_pose_w[1];
  double ego_theta = ego_pose_w[2];

  double dx = pose_w[0] - ego_x;
  double dy = pose_w[1] - ego_y;

  double cos_theta = std::cos(ego_theta);
  double sin_theta = std::sin(ego_theta);

  double x = dx * cos_theta + dy * sin_theta;
  double y = -dx * sin_theta + dy * cos_theta;

  double theta = pose_w[2] - ego_theta;
  theta = std::atan2(std::sin(theta), std::cos(theta));

  return Eigen::Vector3d(x, y, theta);
}

Eigen::Vector3d transform_pose_2w(Eigen::Vector3d pose_b, Eigen::Vector3d ego_pose_w) {
  double ego_x = ego_pose_w[0];
  double ego_y = ego_pose_w[1];
  double ego_theta = ego_pose_w[2];

  double x_ego = pose_b[0];
  double y_ego = pose_b[1];
  double theta_ego = pose_b[2];

  double cos_theta = std::cos(ego_theta);
  double sin_theta = std::sin(ego_theta);

  double x_w = x_ego * cos_theta - y_ego * sin_theta;
  double y_w = x_ego * sin_theta + y_ego * cos_theta;

  x_w += ego_x;
  y_w += ego_y;

  double theta_w = theta_ego + ego_theta;
  theta_w = std::atan2(std::sin(theta_w), std::cos(theta_w));

  return Eigen::Vector3d(x_w, y_w, theta_w);
}

double get_distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
  return (p1 - p2).norm();
}

double get_distance(const Eigen::Vector2d &p, const plan_interface::Path &path) {
  if (path.path_points.empty()) {
    throw std::invalid_argument("Path is empty.");
  }

  double min_distance = 10000.0;
  for (size_t i = 0; i < path.path_points.size(); ++i) {
    Eigen::Vector2d path_point(path.path_points[i].x, path.path_points[i].y);
    double distance = (p - path_point).norm();
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

double get_distance(const Eigen::Vector2d &p, const plan_interface::Trajectory &traj) {
  if (traj.trajectory_points.empty()) {
    throw std::invalid_argument("Trajectory is empty.");
  }

  double min_distance = 10000.0;
  for (size_t i = 0; i < traj.trajectory_points.size(); ++i) {
    Eigen::Vector2d traj_point(traj.trajectory_points[i].x, traj.trajectory_points[i].y);
    double distance = (p - traj_point).norm();
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

plan_interface::PathPoint get_closest_path_point(const Eigen::Vector2d &p, const plan_interface::Path &path) {
  if (path.path_points.empty()) {
    throw std::invalid_argument("Path is empty.");
  }

  double min_distance = 10000.0;
  plan_interface::PathPoint closest_path_point;

  for (size_t i = 0; i < path.path_points.size(); ++i) {
    Eigen::Vector2d path_point(path.path_points[i].x, path.path_points[i].y);
    double distance = (p - path_point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_path_point = path.path_points[i];
    }
  }
  return closest_path_point;
}

plan_interface::TrajectoryPoint get_closest_traj_point(const Eigen::Vector2d &p, const plan_interface::Trajectory &traj) {
  if (traj.trajectory_points.empty()) {
    throw std::invalid_argument("Trajectory is empty.");
  }

  double min_distance = 10000.0;
  plan_interface::TrajectoryPoint closest_traj_point;

  for (size_t i = 0; i < traj.trajectory_points.size(); ++i) {
    Eigen::Vector2d traj_point(traj.trajectory_points[i].x, traj.trajectory_points[i].y);
    double distance = (p - traj_point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_traj_point = traj.trajectory_points[i];
    }
  }
  return closest_traj_point;
}

int get_closest_path_point_index(const Eigen::Vector2d &p, const plan_interface::Path &path) {
  if (path.path_points.empty()) {
    throw std::invalid_argument("Path is empty.");
  }
  double min_distance = 10000.0;
  int closest_index = 0;
  for (size_t i = 0; i < path.path_points.size(); ++i) {
    Eigen::Vector2d path_point(path.path_points[i].x, path.path_points[i].y);
    double distance = (p - path_point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

int get_closest_traj_point_index(const Eigen::Vector2d &p, const plan_interface::Trajectory &traj) {
  if (traj.trajectory_points.empty()) {
    throw std::invalid_argument("Trajectory is empty.");
  }

  double min_distance = 10000.0;
  int closest_index = 0;

  for (size_t i = 0; i < traj.trajectory_points.size(); ++i) {
    Eigen::Vector2d traj_point(traj.trajectory_points[i].x, traj.trajectory_points[i].y);
    double distance = (p - traj_point).norm();
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

plan_interface::PathPoint get_intersection_path_point(const Eigen::Vector2d &p, const double &direction,
                                                      const plan_interface::Path &path, int index) {
  if (path.path_points.empty()) {
    throw std::invalid_argument("Path is empty.");
  }
  
  Eigen::Vector2d direction_vec(cos(direction), sin(direction));

  for (size_t i = 0; i < path.path_points.size() - 1; ++i) {
    const plan_interface::PathPoint& p1 = path.path_points[i];
    const plan_interface::PathPoint& p2 = path.path_points[i + 1];
    Eigen::Vector2d segment_direction(p2.x - p1.x, p2.y - p1.y);
    Eigen::Vector2d segment_start(p1.x, p1.y);

    // 求解线性方程：
    // p + t * direction_vec = segment_start + u * segment_direction
    double denominator = direction_vec.x() * segment_direction.y() - direction_vec.y() * segment_direction.x();

    // 射线和线段平行
    if (denominator == 0) continue;

    double t = ((segment_start.x() - p.x()) * segment_direction.y() - (segment_start.y() - p.y()) * segment_direction.x()) / denominator;
    double u = ((segment_start.x() - p.x()) * direction_vec.y() - (segment_start.y() - p.y()) * direction_vec.x()) / denominator;

    //射线与路径线段相交
    if (t >= 0 && u >= 0 && u <= 1) {
      index = i;
      Eigen::Vector2d intersection = p + t * direction_vec;
      return plan_interface::PathPoint(intersection.x(), intersection.y(), atan2(segment_direction.y(), segment_direction.x()));
    }
  }
  // 没有相交点，返回路径的最后一个点
  index = path.path_points.size() - 1;
  return path.path_points.back();
}

std::set<std::pair<int, int>> get_covered_cells_in_map(const std::vector<Eigen::Vector2d>& polygon, 
                                                      const Eigen::Vector2d& map_origin, double &map_reso) {
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
  int min_cell_x = static_cast<int>(std::floor((min_x - map_origin.x()) / map_reso));
  int max_cell_x = static_cast<int>(std::ceil((max_x - map_origin.x()) / map_reso));
  int min_cell_y = static_cast<int>(std::floor((min_y - map_origin.y()) / map_reso));
  int max_cell_y = static_cast<int>(std::ceil((max_y - map_origin.y()) / map_reso));
  for (int cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
      double wx = cell_x * map_reso + map_origin.x();
      double wy = cell_y * map_reso + map_origin.y();
      if (is_point_in_polygon(Eigen::Vector2d(wx, wy), polygon)) {
        covered_cells.emplace(cell_x, cell_y);
      }
    }
  }
  return covered_cells;
}

bool is_point_in_polygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
  // 使用射线法判断点是否在多边形内
  int intersections = 0;
  size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const auto& p1 = polygon[i];
    const auto& p2 = polygon[(i + 1) % n];
    // 检查点是否在线段的垂直投影范围内
    if ((point.y() > std::min(p1.y(), p2.y())) &&
        (point.y() <= std::max(p1.y(), p2.y())) &&
        (point.x() <= std::max(p1.x(), p2.x()))) {
      double x_intersect = p1.x() + (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y());
      if (x_intersect > point.x()) {
        intersections++;
      }
    }
  }
  // 奇数次交点则点在多边形内
  return (intersections % 2) == 1;
}

}  // namespace geometry
}  // namespace utils
