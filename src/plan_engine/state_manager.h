#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <nlohmann/json.hpp>
#include "plan_interface.h"



class StateManager {
public:
  struct Config {
    int max_replan_times;
    int finish_wait_time;
    int standby_wait_time;
    double replan_lat_dev_thold;
    double replan_yaw_dev_thold;
    double arrival_distance_thold;
    double arrival_direction_thold;
    double stop_distance_thold;
    bool is_charge;
    std::vector<double> charge_position;
  };

  StateManager(const nlohmann::json& strategy);
  ~StateManager() = default;

  plan_interface::PlanState get_plan_state() const;

  void glob_plan_decider();

  void set_map_info_ready(const bool is_ready) { map_info_ready_ = is_ready; }
  void set_odom_info_ready(const bool is_ready) { odom_info_ready_ = is_ready; }
  void set_task_info_ready(const bool is_ready) { task_info_ready_ = is_ready; }
  void set_charge_info_ready(const bool is_ready) { charge_task_ready_ = is_ready; }

  void set_goal_ready(const bool is_ready) { goal_ready_ = is_ready; }

  bool get_task_info_ready() const { return task_info_ready_; }
  bool get_goal_ready() const { return goal_ready_; }
  bool get_ready_to_plan() const { return (utils::time_now_ms() - standby_ts_ms_ > static_cast<int64_t>(cfg_.standby_wait_time)); }
  bool is_last_failure() const { return planning_failure_count_ >= cfg_.max_replan_times; }
  bool get_charge_info_ready() const { return charge_task_ready_; }
  bool get_charge_need() const { return cfg_.is_charge; }

  void set_goal_changed(const bool is_changed) { goal_changed_ = is_changed; }
  void set_planning_success(const bool is_success) { planning_success_ = is_success; }
  void set_planning_finished(const bool is_finished) { planning_finished_ = is_finished; }
  void set_mpc_collision(const bool is_collision) { mpc_collision_ = is_collision; }
  
  void set_lat_deviation(const double lat_dev) { lat_deviation_ = lat_dev; }
  void set_yaw_deviation(const double yaw_dev) { yaw_deviation_ = yaw_dev; }
  void set_heading_error(const double heading_error) { heading_error_ = heading_error; }
  void set_stop_state(const bool is_stopped) { is_stopped_ = is_stopped; }
  void set_dist_to_goal(const double dist_to_goal) { dist_to_goal_ = dist_to_goal; }

  std::vector<double> get_charge_position() const { return cfg_.charge_position; }

private:
  void transition_to_state(plan_interface::PlanState new_state);

public:
  Config cfg_;

private:
  plan_interface::PlanState plan_state_{plan_interface::PlanState::NONE};
  bool map_info_ready_{false};
  bool odom_info_ready_{false};
  bool task_info_ready_{false};
  bool goal_ready_{false};
  bool charge_task_ready_{false};

  bool goal_changed_{false};
  bool planning_success_{false};
  bool planning_finished_{false};
  bool mpc_collision_{false};
  bool is_stopped_{false};
  double dist_to_goal_{1000.0};
  double heading_error_{3.0};
  double lat_deviation_{0.0};
  double yaw_deviation_{0.0};

  int planning_failure_count_{0};
  int64_t finish_ts_ms_;
  int64_t standby_ts_ms_;
};

#endif // STATE_MANAGER_H
