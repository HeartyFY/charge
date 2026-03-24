#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H
#include <set>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "plan_interface.h"


struct RobotParams {
  struct Geometry {
    double length; // m
    double width; // m
    double height; // m
    double rear_overhang; // m
  };

  struct Dynamics {
    double max_speed; // m/s
    double max_accel; // m/s^2
    double max_decel; // m/s^2
    double emergency_decel; // m/s^2
    double max_yaw_rate; // rad/s
    double max_yaw_accel; // rad/s^2

    double brake_distance; // m
    double min_safe_distance; // m
    double min_turn_radius; // m

    double static_max_speed; // m/s
    double static_max_yaw_rate; // rad/s
    int static_min_time; // ms

    bool reversable;
    bool rotatable;
  };

  struct Bounding {
    double front_expansion;
    double rear_expansion;
    double side_expansion;

    double loc_map_valid_range;
    double loc_map_x_offset;
  };

  Geometry geom;
  Dynamics dyn;
  Bounding bound;
};

struct RobotState {
  double x;
  double y;
  double yaw;
  double v;
  double omega;
  double remain_s;
  double lat_error;
  double heading_error;
  bool standstill;
  plan_interface::Gear gear;

  RobotState() : x(0), y(0), yaw(0), v(0), omega(0), remain_s(0), standstill(false), gear(plan_interface::Gear::N) {}
};

struct MapMeta {
  int width;
  int height;
  double resolution;
  double origin_x;
  double origin_y;
  double rotation;
};

class RobotModel {
public:
  RobotModel(const std::string &robot_params_file);
  ~RobotModel() = default;

  std::vector<Eigen::Vector2d> get_ego_rectangle(double x, double y, double yaw);
  std::vector<Eigen::Vector2d> get_bounding_rectangle(double x, double y, double yaw);
  std::vector<Eigen::Vector2d> get_bounding_rectangle(Eigen::Vector3d pose, double front_expansion, double rear_expansion, double side_expansion);
  std::set<std::pair<int, int>> get_covered_cells(const std::vector<Eigen::Vector2d>& polygon, 
                                                  const Eigen::Vector3d& map_origin, double resolution);

  std::set<std::pair<int, int>> get_covered_loc_cells(const std::vector<Eigen::Vector2d>& polygon);
  std::set<std::pair<int, int>> get_covered_glob_cells(const std::vector<Eigen::Vector2d>& polygon);
  bool is_point_in_polygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

  void set_glob_map_meta(MapMeta& map_meta) { glob_map_meta_ = map_meta; };
  void set_loc_map_meta(MapMeta& map_meta) { loc_map_meta_ = map_meta; };
  bool is_within_glob_map_bounds(int gx, int gy);
  bool is_within_loc_map_bounds(int lx, int ly);
  bool is_within_valid_loc_map_bounds(int lx, int ly);
  std::pair<int, int> world_to_glob_map_coords(double wx, double wy);
  std::pair<double, double> glob_map_to_world_coords(int gx, int gy);
  std::pair<int, int> world_to_loc_map_coords(double wx, double wy);
  std::pair<double, double> loc_map_to_world_coords(int lx, int ly);
  int get_pos_index_in_glob_map(double wx, double wy);
  int get_map_coord_index_in_glob_map(int gx, int gy);
  int get_pos_index_in_loc_map(double wx, double wy);
  int get_map_coord_index_in_loc_map(int lx, int ly);
  int get_pose_index_in_glob_map(double wx, double wy, double yaw);
  int get_map_coord_index_in_glob_map(int gx, int gy, double yaw);

  inline double get_glob_map_resolution() const { return glob_map_meta_.resolution; }
  inline double get_loc_map_resolution() const { return loc_map_meta_.resolution; }
  inline double get_glob_map_width() const { return glob_map_meta_.width; }
  inline double get_glob_map_height() const { return glob_map_meta_.height; }
  inline double get_loc_map_width() const { return loc_map_meta_.width; }
  inline double get_loc_map_height() const { return loc_map_meta_.height; }


  inline double get_length() const { return params_.geom.length; }
  inline double get_width() const { return params_.geom.width; }
  inline double get_front_overhang() const { return params_.geom.length - params_.geom.rear_overhang; }
  inline double get_rear_overhang() const { return params_.geom.rear_overhang; }
  inline double get_front_expansion() const { return params_.bound.front_expansion; }
  inline double get_rear_expansion() const { return params_.bound.rear_expansion; }
  inline double get_side_expansion() const { return params_.bound.side_expansion; }

  inline double get_max_speed() const { return params_.dyn.max_speed; }
  inline double get_max_accel() const { return params_.dyn.max_accel; }
  inline double get_max_decel() const { return params_.dyn.max_decel; }
  inline double get_emergency_decel() const { return params_.dyn.emergency_decel; }
  inline double get_max_yaw_rate() const { return params_.dyn.max_yaw_rate; }
  inline double get_max_yaw_accel() const { return params_.dyn.max_yaw_accel; }
  inline double get_brake_distance() const { return params_.dyn.brake_distance; }
  inline double get_min_safe_distance() const { return params_.dyn.min_safe_distance; }
  inline double get_min_turn_radius() const { return params_.dyn.min_turn_radius; }
  inline bool is_reversable() const { return params_.dyn.reversable; }
  inline bool is_rotatable() const { return params_.dyn.rotatable; }

  inline void set_state(const RobotState& state) { state_ = state; }
  inline const RobotState& get_state() const { return state_; }
  inline void set_remain_s(double remain_s) { state_.remain_s = remain_s; }
  inline double get_remain_s() const { return state_.remain_s; }
  inline void set_gear(plan_interface::Gear gear) { state_.gear = gear; }
  inline void gear_shift_to(plan_interface::Gear gear) { if (state_.gear == plan_interface::Gear::N) state_.gear = gear; }
  inline plan_interface::Gear get_gear() const { return state_.gear; }
  inline void set_standstill(bool standstill) { state_.standstill = standstill; if (standstill) state_.gear = plan_interface::Gear::N; }
  inline bool get_standstill() const { return state_.standstill; }
  inline void set_lat_error(double lat_error) { state_.lat_error = lat_error; }
  inline double get_lat_error() const { return state_.lat_error; }
  inline void set_heading_error(double heading_error) { state_.heading_error = heading_error; }
  inline double get_heading_error() const { return state_.heading_error; }

  void reset_state();

private:
  void from_file(const std::string &robot_params_file);
  void from_json(const nlohmann::json &robot_params_json);

  Eigen::Vector3d kinematic_diff(const Eigen::Vector3d& state, double v, double w_input);
  Eigen::Vector3d rk4_diff(const Eigen::Vector3d& state, double v, double w_input, double dt);

public:
  std::string robot_name_;
  std::string robot_type_; // Ackermann, Diff, Omni, Bipedal
  RobotParams params_;

private:
  RobotState state_;
  MapMeta glob_map_meta_;
  MapMeta loc_map_meta_;

}; // class RobotModel


#endif // ROBOT_MODEL_H