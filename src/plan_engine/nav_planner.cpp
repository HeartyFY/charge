#include "utils.h"
#include "nav_planner.h"
#include "cmake_config.h"

NavPlanner::NavPlanner(ros::NodeHandle* nh, const NavPlannerConfig& config) {
  msg_handle_ = std::make_shared<MsgHandle>(nh, config.topic_config_file);
  data_source_ = std::make_shared<plan_interface::DataSource>();
  register_msg_callback();
  robot_model_ = std::make_shared<RobotModel>(config.robot_params_file);
  planning_task_ = std::make_shared<PlanningTask>(config.planner_config_file, robot_model_, data_source_);
  
}

NavPlanner::~NavPlanner() { stop_thread(); }

void NavPlanner::Process() {
  while (get_thread_state() == ThreadState::Running) {
    int64_t start_millis = utils::time_now_ms();
    on_running();
    int64_t end_millis = utils::time_now_ms();
    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(static_cast<int64_t>(20) + start_millis - end_millis, static_cast<int64_t>(0))));
  }
}

void NavPlanner::on_running() {
  uint64_t start_time_ms = utils::time_now_ms();

  if (run_once(start_time_ms)) {
    auto control_command = data_source_->output_data.control_command.read();
    if (control_command) {
      msg_handle_->publish_control_command(*control_command);
    } else {
      std::cout << "control_command is null" << std::endl;
    }

    auto plan_debug_info = data_source_->output_data.plan_debug_info.read();
    if (plan_debug_info) {
      msg_handle_->publish_plan_debug_info(*plan_debug_info);
    } else {
      std::cout << "plan_debug_info is null" << std::endl;
    }

    auto navigation_status = data_source_->output_data.navigation_status.read();
    if (navigation_status) {
      msg_handle_->publish_navigation_status(*navigation_status);
    } else {
      std::cout << "navigation_status is null" << std::endl;
    }

    std::cout << "run once success, using : " << utils::time_now_ms() - start_time_ms << "ms" << std::endl;
  } else {
    std::cout << "run once failed" << std::endl;
  }
  
}

bool NavPlanner::run_once(uint64_t start_time_ms) {
  if (planning_task_->update(start_time_ms)) {
    data_source_->output_data.control_command.write(*planning_task_->get_result());
    data_source_->output_data.plan_debug_info.write(*planning_task_->get_debug_info());
    data_source_->output_data.navigation_status.write(*planning_task_->get_nav_status());
    return true;
  } else {
    std::cout << "planning_task update failed" << std::endl;
    return false;
  }
  return true;
}

void NavPlanner::register_msg_callback() {
  msg_handle_->set_map_info_callback([this](const MapInfoMsg::ConstPtr& msg) {
    // std::cout << "map_info received: " << msg->header.stamp << std::endl;
    data_source_->input_data.map_info.write(*msg);
    data_source_->input_data.map_info_updated.write(true);
  });
  msg_handle_->set_odom_info_callback([this](const OdomInfoMsg::ConstPtr& msg) {
    // std::cout << "odom_info received: " << msg->header.stamp << std::endl;
    // std::cout << "odom position x: " << msg->pose.pose.position.x << " y: " << msg->pose.pose.position.y << " z: " << msg->pose.pose.position.z << std::endl;
    data_source_->input_data.odom_info.write(*msg);
    data_source_->input_data.odom_info_updated.write(true);
  });
  msg_handle_->set_obstacle_callback([this](const ObstacleMsg::ConstPtr& msg) {
    // std::cout << "obstacle received: " << msg->header.stamp << std::endl;
    data_source_->input_data.obstacle.write(*msg);
    data_source_->input_data.obstacle_updated.write(true);
  });
  msg_handle_->set_task_info_callback([this](const TaskInfoMsg::ConstPtr& msg) {
    // std::cout << "task_info received: " << msg->header.stamp << std::endl;
    data_source_->input_data.task_info.write(*msg);
    data_source_->input_data.task_info_updated.write(true);
  });
  msg_handle_->set_joystick_callback([this](const JoystickMsg::ConstPtr& msg) {
    // std::cout << "joystick received: " << msg->header.stamp << std::endl;
    data_source_->input_data.joystick.write(*msg);
    data_source_->input_data.joystick_updated.write(true);
  });
}

void NavPlanner::start() {
  if (is_running_ = false) {
    is_running_ = true;
  }
  start_thread();
}

void NavPlanner::stop() {
  if (is_running_ = true) {
    is_running_ = false;
  }
  stop_thread();
}