#include <fstream>

#include "geometry.h"
#include "planning_task.h"
// #include "astar_planner.h"
#include "hybird_astar.h"
// #include "teb_planner.h"
#include "lbfgs_optimizer.h"
#include "mpc_diff_adapter.h"


PlanningTask::PlanningTask(const std::string &planner_config_file, std::shared_ptr<RobotModel> robot_model, std::shared_ptr<plan_interface::DataSource> data_source) 
: robot_model_(robot_model), data_source_(data_source) {

  nlohmann::json strategy_reader, global_config_reader, local_config_reader, actuator_config_reader;
  parse_config(planner_config_file, strategy_reader, global_config_reader, local_config_reader, actuator_config_reader);
  map_grid_ = std::make_shared<GridMap>();
  obst_grid_ = std::make_shared<GridMap>();
  kdtree_ = std::make_shared<utils::kdtree::KDTree2D>();
  state_manager_ = std::make_shared<StateManager>(strategy_reader);
  plan_state_ = state_manager_->get_plan_state();
  global_planner_ = std::make_shared<global_planner::hybird_astar::HybirdAstar>(robot_model, global_config_reader);
  local_planner_ = std::make_shared<local_planner::lbfgs_optimizer::LbfgsOptimizer>(robot_model, local_config_reader);
  actuator_adapter_ = std::make_shared<actuator_adapter::MPCDiffAdapter>(robot_model, actuator_config_reader);

  global_planner_->run();
}

PlanningTask::~PlanningTask() { global_planner_->stop(); }

bool PlanningTask::update(uint64_t start_time_ms) {

  preprocessing(start_time_ms);
  if (((plan_state_ == plan_interface::PlanState::PLANNING || plan_state_ == plan_interface::PlanState::RUNNING) 
      && (robot_model_->get_gear() != plan_interface::Gear::R)) || plan_state_ == plan_interface::PlanState::CHARGE) {
    plan_interface::GlobalPlannerState glob_planner_state = global_planner_->get_state();
    // std::cout << "global planner state: " << static_cast<int>(glob_planner_state) << std::endl;
    if (glob_planner_state == plan_interface::GlobalPlannerState::IDLE) {
      global_planner_->feed_data(map_info_, odom_info_, obstacle_, goal_, robot_model_->get_gear());
      global_planner_->call_update();
      glob_paln_finished_ = false;
      if (plan_state_ == plan_interface::PlanState::CHARGE) {
        global_planner_->set_charge_state(true);
      }
    }
    if (glob_paln_finished_ == false) {
      if (glob_planner_state == plan_interface::GlobalPlannerState::SUCCESS) {
        if (path_update()) {
          glob_paln_success_ = true;
          glob_paln_finished_ = true;
        } else {
          glob_paln_success_ = false;
          glob_paln_finished_ = true;
        }
        collision_mks_ = global_planner_->get_collision_mks();
      }

      if (glob_planner_state == plan_interface::GlobalPlannerState::FAILED) {
        collision_mks_ = global_planner_->get_collision_mks();
        glob_paln_success_ = false;
        glob_paln_finished_ = true;
      }
    }
    // std::cout << "global planner update :" << glob_paln_success_ << ", using : " << utils::time_now_ms() - start_time_ms << "ms" << std::endl;
  }

  if (global_path_->path_points.size() > 0 && glob_paln_success_) {
    local_planner_->feed_data(map_info_, odom_info_, obstacle_, *global_path_);
    bool success = local_planner_->update();
    if (success) {
      local_traj_ = local_planner_->get_result();
    }
  }

  actuator_adapter_->feed_data(odom_info_, *local_traj_);
  bool success = actuator_adapter_->update();
  if (success) {
    control_command_ = actuator_adapter_->get_result();
  }

  wrap_msg();
  return true;
}


void PlanningTask::preprocessing(uint64_t start_time_ms) {

  if (update_map_info()) {
    set_glob_map_meta();
    if (plan_state_ == plan_interface::PlanState::NONE) { state_manager_->set_map_info_ready(true); }
  }

  if (update_odom_info()) {
    if (plan_state_ == plan_interface::PlanState::NONE) { state_manager_->set_odom_info_ready(true); }
    static_check();
  }

  if (update_obstacle()) {
    set_loc_map_meta();
  }

  if (update_joystick()) {}

  if (map_info_updated_ || obstacle_updated_) {
    update_kdtree();
    map_info_updated_ = false;
    obstacle_updated_ = false;
  }


  auto task_info_updated = data_source_->input_data.task_info_updated.read();
  if (task_info_updated.has_value() && task_info_updated.value() == true) {
    data_source_->input_data.task_info_updated.write(false);
    auto cur_task_info = data_source_->input_data.task_info.read();
    if (cur_task_info) {
      if (plan_state_ == plan_interface::PlanState::STANDBY && 
        (!state_manager_->get_task_info_ready() && !cur_task_info->waypoints.empty())) {
        state_manager_->set_task_info_ready(true);
        task_info_ = *cur_task_info;
      }
    }
  }


  if (plan_state_ == plan_interface::PlanState::STANDBY && state_manager_->get_task_info_ready() &&
      state_manager_->get_ready_to_plan()) {
    if (goal_.id == 0) {
      goal_ = task_info_.waypoints.front();
      state_manager_->set_goal_ready(true);
    } else {
      goal_ = task_info_.waypoints[goal_.id];
      state_manager_->set_goal_ready(true);
    }
  }

  if (plan_state_ == plan_interface::PlanState::PLANNING) {
    state_manager_->set_planning_success(glob_paln_success_);
    state_manager_->set_planning_finished(glob_paln_finished_);
  }

  if (global_path_->path_points.size() > 0) {
    Eigen::Vector2d ego_pos(odom_info_.pose.pose.position.x, odom_info_.pose.pose.position.y);
    Eigen::Vector2d goal_pos(goal_.pose.position.x, goal_.pose.position.y);

    state_manager_->set_lat_deviation(robot_model_->get_lat_error());
    state_manager_->set_yaw_deviation(robot_model_->get_heading_error());
    double dist_to_goal = global_path_->is_last_path ? robot_model_->get_remain_s() : 5.0;
    state_manager_->set_dist_to_goal(dist_to_goal);
    state_manager_->set_mpc_collision(trajectory_collision_check());


  }

  // if (plan_state_ == plan_interface::PlanState::FINISHED) {
    
  // }
  if (plan_state_ == plan_interface::PlanState::STANDBY) {
    reset();
    actuator_adapter_->reset();
    robot_model_->reset_state();
  }

  if (state_manager_->get_charge_need()) {
    if (plan_state_ == plan_interface::PlanState::FINISHED) {
      auto charge_position = state_manager_->get_charge_position();
      if (!state_manager_->get_charge_info_ready() && !charge_position.empty()) {
        goal_ = get_charge_pose(charge_position);
        global_path_->path_points.clear();
        state_manager_->set_charge_info_ready(true);
      }
    }
    else if (plan_state_ == plan_interface::PlanState::CHARGE_FINISHED) {
      state_manager_->set_charge_info_ready(false);
      global_planner_->set_charge_state(false);
      if (goal_.id == task_info_.waypoints.size()) {
        goal_ = navi_types::Waypoint();
        state_manager_->set_task_info_ready(false);
        state_manager_->set_goal_ready(false);
      }
    }
  }
  else if ((plan_state_ == plan_interface::PlanState::FAILURE && state_manager_->is_last_failure()) ||
    (plan_state_ == plan_interface::PlanState::FINISHED && goal_.id == task_info_.waypoints.size())) {
    // task_info_ = TaskInfoMsg();
    goal_ = navi_types::Waypoint();
    state_manager_->set_task_info_ready(false);
    state_manager_->set_goal_ready(false);
  }

  state_manager_->glob_plan_decider();
  plan_state_ = state_manager_->get_plan_state();
  collision_mks_.markers.clear();
}

bool PlanningTask::update_map_info() {
  auto map_info_updated = data_source_->input_data.map_info_updated.read();
  if (map_info_updated.has_value() && map_info_updated.value()) {
    data_source_->input_data.map_info_updated.write(false);
    
    if (auto cur_map_info = data_source_->input_data.map_info.read()) {
      map_info_ = *cur_map_info;
      map_info_updated_ = true;
      set_glob_map_meta();
      update_map_points();
      return true;
    }
  }
  return false;
}

bool PlanningTask::update_odom_info() {
  auto odom_info_updated = data_source_->input_data.odom_info_updated.read();
  if (odom_info_updated.has_value() && odom_info_updated.value()) {
    data_source_->input_data.odom_info_updated.write(false);

    if (auto cur_odom_info = data_source_->input_data.odom_info.read()) {
      odom_info_ = *cur_odom_info;
      return true;
    }
  }
  return false;
}

bool PlanningTask::update_task_info() {
  auto task_info_updated = data_source_->input_data.task_info_updated.read();
  if (task_info_updated.has_value() && task_info_updated.value()) {
    data_source_->input_data.task_info_updated.write(false);

    if (auto cur_task_info = data_source_->input_data.task_info.read()) {
      task_info_ = *cur_task_info;
      return true;
    }
  }
  return false;
}

bool PlanningTask::update_obstacle() {
  auto obstacle_updated = data_source_->input_data.obstacle_updated.read();
  if (obstacle_updated.has_value() && obstacle_updated.value()) {
    data_source_->input_data.obstacle_updated.write(false);

    if (auto cur_obstacle = data_source_->input_data.obstacle.read()) {
      obstacle_ = *cur_obstacle;
      obstacle_updated_ = true;
      set_loc_map_meta();
      update_obst_points();
      return true;
    }
  }
  return false;
}

bool PlanningTask::update_joystick() {
  auto joystick_updated = data_source_->input_data.joystick_updated.read();
  if (joystick_updated.has_value() && joystick_updated.value()) {
    data_source_->input_data.joystick_updated.write(false);

    if (auto cur_joystick = data_source_->input_data.joystick.read()) {
      joystick_ = *cur_joystick;
      return true;
    }
  }
  return false;
}

void PlanningTask::set_glob_map_meta() {
  GridMeta map_meta;
  map_meta.width = map_info_.info.width;
  map_meta.height = map_info_.info.height;
  map_meta.resolution = map_info_.info.resolution;
  map_meta.origin_x = map_info_.info.origin.position.x;
  map_meta.origin_y = map_info_.info.origin.position.y;
  map_meta.rotation = utils::geometry::get_yaw_from_ros_q(map_info_.info.origin.orientation);
  map_grid_->set_meta(map_meta);
}

void PlanningTask::set_loc_map_meta() {
  int width = obstacle_.info.width;
  int height = obstacle_.info.height;
  double resolution = obstacle_.info.resolution;
  double rotation = utils::geometry::get_yaw_from_ros_q(obstacle_.info.origin.orientation);
  
  GridMeta map_meta;
  map_meta.width = width;
  map_meta.height = height;
  map_meta.resolution = resolution;
  map_meta.origin_x = obstacle_.info.origin.position.x - (std::cos(rotation)*width*resolution/2 - std::sin(rotation)*height*resolution/2);
  map_meta.origin_y = obstacle_.info.origin.position.y - (std::sin(rotation)*width*resolution/2 + std::cos(rotation)*height*resolution/2);
  map_meta.rotation = rotation;
  obst_grid_->set_meta(map_meta);
}

void PlanningTask::update_map_points() {
  std::vector<Eigen::Vector2d> points;
  int glob_width = map_info_.info.width;
  int glob_height = map_info_.info.height;
  for (int x = 0; x < glob_width; x++) {
    for (int y = 0; y < glob_height; y++) {
      if (x == 0 || x == glob_width-1 || y == 0 || y == glob_height-1 || (map_info_.data[y * glob_width + x] > 0)) {
        Eigen::Vector2d point = map_grid_->grid_to_world(Eigen::Vector2i(x, y));
        points.emplace_back(point);
      }
    }
  }
  map_points_ = points;
}

void PlanningTask::update_obst_points() {
  std::vector<Eigen::Vector2d> points;
  int local_width = obstacle_.info.width;
  int local_height = obstacle_.info.height;

  for (int x = 0; x < local_width; x++) {
    for (int y = 0; y < local_height; y++) {
      if (obstacle_.data[y * local_width + x].occupancy) {
        Eigen::Vector2d point = obst_grid_->grid_to_world(Eigen::Vector2i(x, y));
        points.emplace_back(point);
      }
    }
  }
  obst_points_ = points;
}

void PlanningTask::update_kdtree() {
  std::vector<Eigen::Vector2d> points;
  points.reserve(map_points_.size() + obst_points_.size());
  points.insert(points.end(), map_points_.begin(), map_points_.end());
  points.insert(points.end(), obst_points_.begin(), obst_points_.end());

  if (points.empty()) {
    points.emplace_back(Eigen::Vector2d(1000, 1000));
  }

  kdtree_->init(points);
}

void PlanningTask::static_check() {
  if ((std::abs(odom_info_.twist.twist.linear.x) < robot_model_->params_.dyn.static_max_speed &&
    std::abs(odom_info_.twist.twist.angular.z) < robot_model_->params_.dyn.static_max_yaw_rate) &&
    (emergency_brake_ || robot_model_->get_remain_s() < robot_model_->get_brake_distance())) {
    static_cout_ ++;
    if (static_cout_ > 1.0e+4) static_cout_ = 1.0e+4;
  } else {
    static_cout_ = 0;
  }
  if (static_cout_ > robot_model_->params_.dyn.static_min_time/20) {
    state_manager_->set_stop_state(true);
    robot_model_->set_standstill(true);
  } else {
    state_manager_->set_stop_state(false);
    robot_model_->set_standstill(false);
  }
}

bool PlanningTask::path_update() {
  std::shared_ptr<plan_interface::Path> new_path = global_planner_->get_result();
  double distance_to_teminal = std::hypot(odom_info_.pose.pose.position.x - new_path->path_points.back().x, 
                                          odom_info_.pose.pose.position.y - new_path->path_points.back().y);
  double new_path_length = 0.0;
  for (int i = 1; i < new_path->path_points.size()-1; i++) {
    new_path_length += std::hypot(new_path->path_points[i].x - new_path->path_points[i+1].x, 
                                  new_path->path_points[i].y - new_path->path_points[i+1].y);
  }
  std::cout << "new path length: " << new_path_length << "; distance to teminal: " << distance_to_teminal << "; times: " << new_path_length/distance_to_teminal << std::endl;
  if (new_path_length/distance_to_teminal > 3.0) {
    return false;
  }
  robot_model_->gear_shift_to(new_path->path_points.front().gear);
  global_path_ = new_path;
  return true;
}

void PlanningTask::reset() {
  glob_paln_success_ = false;
  glob_paln_finished_ = true;
  safe_distance_ = 1.0;
  global_path_ = std::make_shared<plan_interface::Path>();
  local_traj_ = std::make_shared<plan_interface::Trajectory>();
  control_command_ = std::make_shared<ControlCommandMsg>();
  plan_debug_info_ = std::make_shared<PlanDebugInfoMsg>();
  state_manager_->set_dist_to_goal(5.0);
}

void PlanningTask::wrap_msg() {

  if (emergency_brake_) {
    control_command_->linear.x = 0.0;
    control_command_->angular.z = 0.0;
  }

  nav_status_->header.stamp = ros::Time::now();
  nav_status_->state.value = static_cast<uint8_t>(plan_state_);

  std::shared_ptr<PlanDebugInfoMsg> cur_plan_debug_info = std::make_shared<PlanDebugInfoMsg>();
  cur_plan_debug_info->timestamp = utils::time_now_ms();
  cur_plan_debug_info->gear.value = static_cast<uint8_t>(robot_model_->get_gear());
  cur_plan_debug_info->remain_s = robot_model_->get_remain_s();
  cur_plan_debug_info->safe_distance = safe_distance_;
  cur_plan_debug_info->emergency_brake = emergency_brake_;
  cur_plan_debug_info->lat_error = robot_model_->get_lat_error() * 100.0;
  cur_plan_debug_info->heading_error = robot_model_->get_heading_error() * 57.3;

  cur_plan_debug_info->global_planner.target_pose = goal_.pose;
  for (int i = 0; i < global_path_->path_points.size(); i++) {
    geometry_msgs::Pose pose_i;
    pose_i.position.x = global_path_->path_points[i].x;
    pose_i.position.y = global_path_->path_points[i].y;
    pose_i.orientation = utils::geometry::get_ros_q_from_yaw(global_path_->path_points[i].yaw);
    cur_plan_debug_info->global_planner.path.emplace_back(pose_i);
  }

  cur_plan_debug_info->local_planner.target_pose = goal_.pose;
  for (int i = 0; i < local_traj_->trajectory_points.size(); i++) {
    geometry_msgs::Pose pose_i;
    pose_i.position.x = local_traj_->trajectory_points[i].x;
    pose_i.position.y = local_traj_->trajectory_points[i].y;
    pose_i.orientation = utils::geometry::get_ros_q_from_yaw(local_traj_->trajectory_points[i].yaw);
    cur_plan_debug_info->local_planner.path.emplace_back(pose_i);
  }

  auto ref_traj = actuator_adapter_->get_ref_trajectory();
  for (int i = 0; i < ref_traj->trajectory_points.size(); i++) {
    msg_interface::MPCState state_i;
    state_i.x = ref_traj->trajectory_points[i].x;
    state_i.y = ref_traj->trajectory_points[i].y;
    state_i.theta = ref_traj->trajectory_points[i].theta;
    state_i.v = ref_traj->trajectory_points[i].v;
    state_i.omega = ref_traj->trajectory_points[i].omega;
    cur_plan_debug_info->ref_trajectory.emplace_back(state_i);
  }

  auto mpc_traj = actuator_adapter_->get_mpc_trajectory();
  for (int i = 0; i < mpc_traj->trajectory_points.size(); i++) {
    Eigen::Vector3d cur_pose(odom_info_.pose.pose.position.x, odom_info_.pose.pose.position.y, utils::geometry::get_yaw_from_ros_q(odom_info_.pose.pose.orientation));
    Eigen::Vector3d traj_pose_b(mpc_traj->trajectory_points[i].x, mpc_traj->trajectory_points[i].y, mpc_traj->trajectory_points[i].theta);
    Eigen::Vector3d traj_pose_w = utils::geometry::transform_pose_2w(traj_pose_b, cur_pose);
    msg_interface::MPCState state_i;
    state_i.x = traj_pose_w(0);
    state_i.y = traj_pose_w(1);
    state_i.theta = traj_pose_w(2);
    state_i.v = mpc_traj->trajectory_points[i].v;
    state_i.omega = mpc_traj->trajectory_points[i].omega;
    state_i.a = mpc_traj->trajectory_points[i].a;
    state_i.alpha = mpc_traj->trajectory_points[i].alpha;
    cur_plan_debug_info->mpc_trajectory.emplace_back(state_i);
  }

  cur_plan_debug_info->global_planner.collision_markers = collision_mks_;

  plan_debug_info_ = cur_plan_debug_info;
}

bool PlanningTask::trajectory_collision_check() {
  double ego_x = odom_info_.pose.pose.position.x;
  double ego_y = odom_info_.pose.pose.position.y;
  double ego_yaw = utils::geometry::get_yaw_from_ros_q(odom_info_.pose.pose.orientation);
  

  auto mpc_traj = actuator_adapter_->get_mpc_trajectory();
  if (mpc_traj->trajectory_points.size() > 0) {
    safe_distance_ = 0.0;
    emergency_brake_ = false;
    Eigen::Vector3d ego_pose_w(ego_x, ego_y, ego_yaw);
    for (int i = 1; i < mpc_traj->trajectory_points.size(); i += 1) {
      double x = mpc_traj->trajectory_points[i].x;
      double y = mpc_traj->trajectory_points[i].y;
      double yaw = mpc_traj->trajectory_points[i].theta;
      Eigen::Vector3d pose_b(x, y, yaw);
      Eigen::Vector3d pose_w = utils::geometry::transform_pose_2w(pose_b, ego_pose_w);
      if (i > 1) {
        safe_distance_ += std::hypot(x - mpc_traj->trajectory_points[i-1].x, y - mpc_traj->trajectory_points[i-1].y);
      }
      if (!collision_free(pose_w) && safe_distance_ < robot_model_->get_min_safe_distance()) {
        emergency_brake_ = true;
        return true;
      }   
    }
  }

  if (local_traj_->trajectory_points.size() > 0) {
    double local_safe_distance_ = 0.0;
    Eigen::Vector2d cur_pos(ego_x, ego_y);
    int start_index = utils::geometry::get_closest_traj_point_index(cur_pos, *local_traj_);
    start_index = std::max(0, start_index - 1);
    int end_index = std::min(start_index + 31, static_cast<int>(local_traj_->trajectory_points.size()));
    for (int i = start_index; i < end_index; i += 2) {
      double x = local_traj_->trajectory_points[i].x;
      double y = local_traj_->trajectory_points[i].y;
      double yaw = local_traj_->trajectory_points[i].yaw;
      if (i > start_index) {
        local_safe_distance_ += std::hypot(x - local_traj_->trajectory_points[i-2].x, y - local_traj_->trajectory_points[i-2].y);
      }

      if (!collision_free(Eigen::Vector3d(x, y, yaw))) {
        return true;
      }
    }
  }

  return false;
}

bool PlanningTask::collision_free(const Eigen::Vector3d &pose) {
  auto [dist, obst] = kdtree_->get_near_with_dist(pose.head<2>());

  std::vector<Eigen::Vector2d> bounding_polygon = robot_model_->get_bounding_rectangle(pose[0], pose[1], pose[2]);
  if (!robot_model_->is_point_in_polygon(obst, bounding_polygon)) {
    return true;
  }

  std_msgs::ColorRGBA red; // rgb(255, 0, 0)
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 0.5;

  visualization_msgs::Marker polygon;
  polygon.header.frame_id = "map";
  polygon.id = 0;
  polygon.ns = "collision_polygon";
  polygon.action = visualization_msgs::Marker::ADD;
  polygon.type = visualization_msgs::Marker::LINE_STRIP;
  polygon.scale.x = 0.02;
  polygon.pose.orientation.w = 1.0;

  polygon.points.clear();
  for (auto &point : bounding_polygon) {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    polygon.points.emplace_back(p);
  }
  polygon.points.emplace_back(polygon.points[0]);
  polygon.color = red;
  polygon.id = polygon.id + 1;
  collision_mks_.markers.emplace_back(polygon);
  
  return false;
}

bool PlanningTask::is_path_stable(std::shared_ptr<plan_interface::Path> new_path) {
  if (global_path_->path_points.empty()) {
    return true;
  }
  // global_path_->path_points.back()
  if (std::hypot(new_path->path_points.back().x - global_path_->path_points.back().x,
                new_path->path_points.back().y - global_path_->path_points.back().y) < 0.1 &&
      std::abs(new_path->path_points.back().yaw - global_path_->path_points.back().yaw) < 0.1) {

    int closest_index = utils::geometry::get_closest_path_point_index(
                        Eigen::Vector2d(new_path->path_points.front().x, new_path->path_points.front().y), *global_path_);
    int end_index = std::min(static_cast<int>(new_path->path_points.size()), static_cast<int>(global_path_->path_points.size())-closest_index);
    double total_displacement = 0.0;
    for (int i = 0; i < end_index; ++i) {
      total_displacement += std::hypot(new_path->path_points[i].x - global_path_->path_points[closest_index+i].x,
                                       new_path->path_points[i].y - global_path_->path_points[closest_index+i].y);
    }
    if (total_displacement / end_index < 1.0) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

std::shared_ptr<ControlCommandMsg> PlanningTask::get_result() {
  return control_command_;
}
std::shared_ptr<PlanDebugInfoMsg> PlanningTask::get_debug_info() {
  return plan_debug_info_;
}
std::shared_ptr<NavigationStatusMsg> PlanningTask::get_nav_status() {
  return nav_status_;
}

void PlanningTask::parse_config(const std::string &planner_config_file, nlohmann::json &strategy_reader,
                    nlohmann::json &global_config_reader, nlohmann::json &local_config_reader,
                    nlohmann::json &actuator_config_reader) {
  std::ifstream json_stream(planner_config_file);
  if (!json_stream.is_open()) {
    throw std::runtime_error("Unable to open planner_config_file: " + planner_config_file);
  }
  try {
    nlohmann::json planner_config_json = nlohmann::json::parse(json_stream);
    strategy_reader = planner_config_json.at("strategy");
    global_config_reader = planner_config_json.at("hybird_astar");
    local_config_reader = planner_config_json.at("lbfgs_opter");
    actuator_config_reader = planner_config_json.at("actuator_adapter");
  } catch (const std::exception& e) {
    throw std::runtime_error("Unable to parse planner_config_file: " + planner_config_file);
  }
  json_stream.close();
}

navi_types::Waypoint PlanningTask::get_charge_pose(const std::vector<double> &position) {
  navi_types::Waypoint pose;
  pose.action = goal_.action;
  pose.audio = goal_.audio;
  pose.id = goal_.id;
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = 0.0;
  pose.pose.orientation = utils::geometry::get_ros_q_from_yaw(position[2]);

  return pose;
}