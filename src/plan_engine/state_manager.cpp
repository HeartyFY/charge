#include "utils.h"
#include "state_manager.h"
#include <iostream>

StateManager::StateManager(const nlohmann::json& strategy) {
  cfg_.max_replan_times = strategy.contains("max_replan_times") ? strategy.at("max_replan_times").get<int>() : 3;
  cfg_.finish_wait_time = strategy.contains("finish_wait_time") ? strategy.at("finish_wait_time").get<int>() : 2000;
  cfg_.standby_wait_time = strategy.contains("standby_wait_time") ? strategy.at("standby_wait_time").get<int>() : 4000;
  cfg_.replan_lat_dev_thold = strategy.contains("replan_lat_dev_thold") ? strategy.at("replan_lat_dev_thold").get<double>() : 3.0;
  cfg_.replan_yaw_dev_thold = strategy.contains("replan_yaw_dev_thold")?  strategy.at("replan_yaw_dev_thold").get<double>()/57.3 : 90.0/57.3;
  cfg_.arrival_distance_thold = strategy.contains("arrival_distance_thold")?  strategy.at("arrival_distance_thold").get<double>() : 0.5;
  cfg_.arrival_direction_thold = strategy.contains("arrival_direction_thold")?  strategy.at("arrival_direction_thold").get<double>() : 5.0/57.3;
  cfg_.stop_distance_thold = strategy.contains("stop_distance_thold")?  strategy.at("stop_distance_thold").get<double>() : 1.5;
  cfg_.is_charge = strategy.contains("is_charge")?  strategy.at("is_charge").get<bool>() : false;

  std::vector<double> tmp_vec;
  tmp_vec = strategy.contains("charge_position")?  strategy.at("charge_position").get<std::vector<double>>() : std::vector<double>();
  cfg_.charge_position.resize(tmp_vec.size());
  for (u_int i = 0; i < tmp_vec.size(); ++i)
  {
    cfg_.charge_position[i] = tmp_vec[i];
  }

  std::cout << "StateManager initialized" << std::endl;
  std::cout << "cfg_.max_replan_times: " << cfg_.max_replan_times << std::endl;
  std::cout << "cfg_.finish_wait_time: " << cfg_.finish_wait_time << std::endl;
  std::cout << "cfg_.standby_wait_time: " << cfg_.standby_wait_time << std::endl;
  std::cout << "cfg_.replan_lat_dev_thold: " << cfg_.replan_lat_dev_thold << std::endl;
  std::cout << "cfg_.replan_yaw_dev_thold: " << cfg_.replan_yaw_dev_thold << std::endl;
  std::cout << "cfg_.arrival_distance_thold: " << cfg_.arrival_distance_thold << std::endl;
  std::cout << "cfg_.arrival_direction_thold: " << cfg_.arrival_direction_thold << std::endl;
  std::cout << "cfg_.stop_distance_thold: " << cfg_.stop_distance_thold << std::endl;
  std::cout << "cfg_.is_charge: " << cfg_.is_charge << std::endl;

  standby_ts_ms_ = utils::time_now_ms();
}

plan_interface::PlanState StateManager::get_plan_state() const {
  return plan_state_;
}

void StateManager::glob_plan_decider() {
  switch (plan_state_) {
    case plan_interface::PlanState::NONE:
      if (/*map_info_ready_ && */odom_info_ready_) {
        transition_to_state(plan_interface::PlanState::STANDBY);
      }
      break;

    case plan_interface::PlanState::STANDBY:
      if (task_info_ready_ && goal_ready_ && 
        (utils::time_now_ms() - standby_ts_ms_ > static_cast<int64_t>(cfg_.standby_wait_time))) {
        transition_to_state(plan_interface::PlanState::PLANNING);
      }
      break;

    case plan_interface::PlanState::PLANNING: {
      if (dist_to_goal_ < cfg_.stop_distance_thold) {
        transition_to_state(plan_interface::PlanState::STOPPING);
      }
      if (planning_finished_) {
        if (planning_success_ ) {
          transition_to_state(plan_interface::PlanState::RUNNING);
        } else {
          transition_to_state(plan_interface::PlanState::FAILURE);
        }
      }
      break;
    }

    case plan_interface::PlanState::RUNNING: 
      if (lat_deviation_ > cfg_.replan_lat_dev_thold || goal_changed_ || mpc_collision_) {
        transition_to_state(plan_interface::PlanState::PLANNING);
        set_goal_changed(false);
        set_mpc_collision(false);
      } else if (dist_to_goal_ < cfg_.stop_distance_thold) {
        transition_to_state(plan_interface::PlanState::STOPPING);
      }
      break;
    case plan_interface::PlanState::STOPPING:
      if ((dist_to_goal_ < cfg_.arrival_distance_thold && 
        std::abs(yaw_deviation_) < cfg_.arrival_direction_thold) || is_stopped_ || mpc_collision_) {
        set_mpc_collision(false);
        finish_ts_ms_ = utils::time_now_ms();
        transition_to_state(plan_interface::PlanState::FINISHED);
      }
      break;
    case plan_interface::PlanState::FAILURE:
      if (dist_to_goal_ < cfg_.stop_distance_thold) {
        transition_to_state(plan_interface::PlanState::STOPPING);
      }
      if (planning_failure_count_ >= cfg_.max_replan_times) {
        transition_to_state(plan_interface::PlanState::STANDBY);
        set_lat_deviation(0.0);
        set_yaw_deviation(0.0);
        set_dist_to_goal(1000.0);
        set_heading_error(3.14);
      } else {
        transition_to_state(plan_interface::PlanState::PLANNING);
      }
      break;

    case plan_interface::PlanState::FINISHED:
      if (utils::time_now_ms() - finish_ts_ms_ > static_cast<int64_t>(cfg_.finish_wait_time)) {
        if (cfg_.is_charge && get_charge_info_ready())
          transition_to_state(plan_interface::PlanState::CHARGE);
        else {
          transition_to_state(plan_interface::PlanState::STANDBY);
          standby_ts_ms_ = utils::time_now_ms();
        }
        set_lat_deviation(0.0);
        set_yaw_deviation(0.0);
        set_dist_to_goal(1000.0);
        set_heading_error(3.14);
      }
      break;

    case plan_interface::PlanState::CHARGE:
      if ((dist_to_goal_ < cfg_.arrival_distance_thold &&
          std::abs(yaw_deviation_) < cfg_.arrival_direction_thold)) {
        // std::cout << "charge output: " << dist_to_goal_ << ", " << yaw_deviation_ << ", " << mpc_collision_ << std::endl;
        finish_ts_ms_ = utils::time_now_ms();
        transition_to_state(plan_interface::PlanState::CHARGE_FINISHED);
      }
      break;

    case plan_interface::PlanState::CHARGE_FINISHED:
      if (!get_charge_info_ready() && utils::time_now_ms() - finish_ts_ms_ > static_cast<int64_t>(cfg_.finish_wait_time)) {
        transition_to_state(plan_interface::PlanState::STANDBY);
        set_lat_deviation(0.0);
        set_yaw_deviation(0.0);
        set_dist_to_goal(1000.0);
        set_heading_error(3.14);
        standby_ts_ms_ = utils::time_now_ms();
      }
      break;

    default:
      break;
  }
}

void StateManager::transition_to_state(plan_interface::PlanState new_state) {
  if (plan_state_ != new_state) {
    std::cout << "Transitioning from " << static_cast<int>(plan_state_) << " to " << static_cast<int>(new_state) << std::endl;
    plan_state_ = new_state;

    if (new_state == plan_interface::PlanState::FAILURE) {
      planning_failure_count_++;
    } else if (new_state == plan_interface::PlanState::STANDBY || new_state == plan_interface::PlanState::RUNNING) {
      planning_failure_count_ = 0;
    }
  }
}
