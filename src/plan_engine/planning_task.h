#ifndef PLANNING_TASK_H
#define PLANNING_TASK_H

#include "utils.h"
#include "ros_msg.h"
#include "double_buffer.h"
#include "kdtree.h"
#include "grid_map.h"
#include "robot_model.h"
#include "state_manager.h"
#include "global_planner.h"
#include "local_planner.h"
#include "actuator_adapter.h"

#include <geometry_msgs/Pose.h>


class PlanningTask {
public:
  PlanningTask(const std::string &planner_config_file, std::shared_ptr<RobotModel> robot_model, std::shared_ptr<plan_interface::DataSource> data_source);
  virtual ~PlanningTask();

  bool update(uint64_t start_time_ms);

  std::shared_ptr<ControlCommandMsg> get_result();
  std::shared_ptr<PlanDebugInfoMsg> get_debug_info();
  std::shared_ptr<NavigationStatusMsg> get_nav_status();


private:
  bool update_map_info();
  bool update_odom_info();
  bool update_task_info();
  bool update_obstacle();
  bool update_joystick();
  void set_glob_map_meta();
  void set_loc_map_meta();
  void update_map_points();
  void update_obst_points();
  void update_kdtree();

  bool collision_free(const Eigen::Vector3d &pose);
  void static_check();
  bool path_update();

  void preprocessing(uint64_t start_time_ms);
  bool trajectory_collision_check();
  void wrap_msg();
  bool is_path_stable(std::shared_ptr<plan_interface::Path> new_path);
  void reset();
  void parse_config(const std::string &planner_config_file, nlohmann::json &strategy_reader,
                    nlohmann::json &global_config_reader, nlohmann::json &local_config_reader,
                    nlohmann::json &actuator_config_reader);
  
  
  navi_types::Waypoint get_charge_pose(const std::vector<double> &position);

private:
  bool glob_paln_success_{false};
  bool glob_paln_finished_{false};
  bool map_info_updated_{false};
  bool obstacle_updated_{false};
  bool emergency_brake_{false};
  double safe_distance_{1.0};
  int static_cout_{0};
  std::shared_ptr<GridMap> map_grid_, obst_grid_;
  std::vector<Eigen::Vector2d> map_points_, obst_points_;
  std::shared_ptr<utils::kdtree::KDTree2D> kdtree_;

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<StateManager> state_manager_;
  std::shared_ptr<plan_interface::DataSource> data_source_;
  std::shared_ptr<global_planner::GlobalPlanner> global_planner_;
  std::shared_ptr<local_planner::LocalPlanner> local_planner_;
  std::shared_ptr<actuator_adapter::ActuatorAdapter> actuator_adapter_;

  MapInfoMsg map_info_;
  OdomInfoMsg odom_info_;
  TaskInfoMsg task_info_;
  ObstacleMsg obstacle_;
  JoystickMsg joystick_;
  navi_types::Waypoint goal_;
  navi_types::Waypoint charge_goal_;

  std::shared_ptr<ControlCommandMsg> control_command_ = std::make_shared<ControlCommandMsg>();
  std::shared_ptr<PlanDebugInfoMsg> plan_debug_info_ = std::make_shared<PlanDebugInfoMsg>();
  std::shared_ptr<NavigationStatusMsg> nav_status_ = std::make_shared<NavigationStatusMsg>();

  plan_interface::PlanState plan_state_;
  std::shared_ptr<plan_interface::Path> global_path_ = std::make_shared<plan_interface::Path>();
  std::shared_ptr<plan_interface::Trajectory> local_traj_ = std::make_shared<plan_interface::Trajectory>();

  visualization_msgs::MarkerArray collision_mks_;

  geometry_msgs::Pose charge_pose_;
};  


#endif // PLANNING_TASK_H