#ifndef UTILS_GEOMETRY_H
#define UTILS_GEOMETRY_H

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include "plan_interface.h"

namespace utils {
namespace geometry {

double clamp(double value, double min_value, double max_value);

double normalize_angle(double angle_rad); // 将角度转换到 [-pi, pi] 范围内

double get_yaw_from_ros_q(const geometry_msgs::Quaternion &q);

geometry_msgs::Quaternion get_ros_q_from_yaw(const double &yaw);

// 坐标转换
Eigen::Vector3d transform_pose_2b(Eigen::Vector3d pose_w, Eigen::Vector3d ego_pose_w);
Eigen::Vector3d transform_pose_2w(Eigen::Vector3d pose_b, Eigen::Vector3d ego_pose_w);

// 计算两点之间的欧几里得距离
double get_distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

// 计算点到路径的最短距离
double get_distance(const Eigen::Vector2d &p, const plan_interface::Path &path);

// 计算点到轨迹的最短距离
double get_distance(const Eigen::Vector2d &p, const plan_interface::Trajectory &traj);

// 获取路径上离指定点最近的点
plan_interface::PathPoint get_closest_path_point(const Eigen::Vector2d &p, const plan_interface::Path &path);

// 获取轨迹上离指定点最近的点
plan_interface::TrajectoryPoint get_closest_traj_point(const Eigen::Vector2d &p, const plan_interface::Trajectory &traj);

// 获取路径上离指定点最近的点的索引
int get_closest_path_point_index(const Eigen::Vector2d &p, const plan_interface::Path &path);

// 获取轨迹上离指定点最近的点的索引
int get_closest_traj_point_index(const Eigen::Vector2d &p, const plan_interface::Trajectory &traj);

// 获取射线与路径相交的点
plan_interface::PathPoint get_intersection_path_point(const Eigen::Vector2d &p, const double &direction,
                                                 const plan_interface::Path &path, int index);

// 获取栅格地图上被多边形覆盖的栅格索引
std::set<std::pair<int, int>> get_covered_cells_in_map(const std::vector<Eigen::Vector2d>& polygon, 
                                                      const Eigen::Vector2d& map_origin, double &map_reso);
// 判断点是否在多边形内部
bool is_point_in_polygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

}  // namespace geometry
}  // namespace utils

#endif  // UTILS_GEOMETRY_H
