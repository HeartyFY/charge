#include "mpc_diff_adapter.h"
#include "geometry.h"
#include "spline.h"
namespace actuator_adapter
{

MPCDiffAdapter::MPCDiffAdapter(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & actuator_config_reader)
  : robot_model_(robot_model), 
  control_command_(std::make_shared<ControlCommandMsg>()),
  ref_trajectory_(std::make_shared<plan_interface::MPCTrajectory>()),
  mpc_trajectory_(std::make_shared<plan_interface::MPCTrajectory>()) {
  parse_config_json(actuator_config_reader);
  mpc_solver_ = std::make_shared<MPCSolver>();
  set_mpc_bounds();
  set_mpc_weights();
}

void MPCDiffAdapter::parse_config_json(const nlohmann::json & actuator_config_reader) {
  cfg_.target_vel = actuator_config_reader.contains("target_vel") ? actuator_config_reader.at("target_vel").get<double>() : 0.3;
  cfg_.control_horizon_index = actuator_config_reader.contains("control_horizon_index") ? actuator_config_reader.at("control_horizon_index").get<int>() : 1;
  

  std::cout << "MPCDiffAdapter initialized" << std::endl;
  std::cout << "cfg_.target_vel: " << cfg_.target_vel << std::endl;
  std::cout << "cfg_.control_horizon_index: " << cfg_.control_horizon_index << std::endl;
}

std::shared_ptr<ControlCommandMsg> MPCDiffAdapter::get_result() {
  return control_command_;
}
std::shared_ptr<plan_interface::MPCTrajectory> MPCDiffAdapter::get_mpc_trajectory() { 
  return mpc_trajectory_;
} 
std::shared_ptr<plan_interface::MPCTrajectory> MPCDiffAdapter::get_ref_trajectory() { 
  return ref_trajectory_; 
}

void MPCDiffAdapter::feed_data(const OdomInfoMsg& odom, const plan_interface::Trajectory& local_traj) {
  current_odom_ = odom;
  current_local_traj_ = local_traj;
}

bool MPCDiffAdapter::update() {
  if (current_local_traj_.trajectory_points.empty()) {
    reset();
    return false;
  }
  MPCInput mpc_input;
  pretreating(mpc_input);

  auto success  = mpc_solver_->update(mpc_input);
  if (success) {
    MPCOutput mpc_output = mpc_solver_->get_output();
    double v_cmd = mpc_output.v_mpc_vec_[cfg_.control_horizon_index];
    double omega_cmd = mpc_output.omega_mpc_vec_[cfg_.control_horizon_index];
    if (remain_s_ < robot_model_->get_brake_distance()) {
      v_cmd = 0.0;
      if (!current_local_traj_.is_last_trajectory) {
        omega_cmd = 0.0;
      }
    }
    apply_limits(v_cmd, omega_cmd);
    set_mpc_trajectory(mpc_output);
    return true;
  } else {
    std::cout << "MPC solve failed" << std::endl;
    return false;
  }
}

void MPCDiffAdapter::reset() {
  control_command_->linear.x = 0.0;
  control_command_->angular.z = 0.0;
}

void MPCDiffAdapter::set_mpc_trajectory(const MPCOutput &mpc_output) {
  std::shared_ptr<plan_interface::MPCTrajectory> cur_mpc_trajectory = std::make_shared<plan_interface::MPCTrajectory>();
  for (int i = 0; i < mpc_output.x_mpc_vec_.size(); i++) {
    plan_interface::MPCState state;
    state.x = mpc_output.x_mpc_vec_[i];
    state.y = mpc_output.y_mpc_vec_[i];
    state.theta = utils::geometry::normalize_angle(mpc_output.theta_mpc_vec_[i]);
    state.v = mpc_output.v_mpc_vec_[i];
    state.omega = mpc_output.omega_mpc_vec_[i];
    state.a = mpc_output.a_mpc_vec_[i];
    state.alpha = mpc_output.alpha_mpc_vec_[i];
    cur_mpc_trajectory->trajectory_points.emplace_back(state);
  }
  mpc_trajectory_ = cur_mpc_trajectory;
}

void MPCDiffAdapter::set_mpc_bounds() {
  Bounds bounds;
  bounds.v_lower_bound = -robot_model_->get_max_speed();
  bounds.v_upper_bound = robot_model_->get_max_speed();
  bounds.omega_lower_bound = -robot_model_->get_max_yaw_rate();
  bounds.omega_upper_bound = robot_model_->get_max_yaw_rate();
  bounds.a_lower_bound = -robot_model_->get_max_decel();
  bounds.a_upper_bound = robot_model_->get_max_accel();
  bounds.alpha_lower_bound = -robot_model_->get_max_yaw_accel();
  bounds.alpha_upper_bound = robot_model_->get_max_yaw_accel();
  mpc_solver_->set_bounds(bounds);
}

void MPCDiffAdapter::set_mpc_weights() {
  Weights weights;
  weights.q_x = 1.0;
  weights.q_y = 1.0;
  weights.q_theta = 1.0;
  weights.q_v = 0.1;
  weights.q_omega = 0.1;

  weights.q_x_n = 100.0;
  weights.q_y_n = 100.0;
  weights.q_theta_n = 100.0;
  weights.q_v_n = 100.0;
  weights.q_omega_n = 10.0;

  weights.r_a = 0.5;
  weights.r_alpha = 0.5;

  mpc_solver_->set_weights(weights);
}

void MPCDiffAdapter::pretreating(MPCInput &mpc_input) {
  double ego_x = current_odom_.pose.pose.position.x;
  double ego_y = current_odom_.pose.pose.position.y;
  double ego_theta = utils::geometry::get_yaw_from_ros_q(current_odom_.pose.pose.orientation);
  Eigen::Vector2d cur_pos(ego_x, ego_y);
  Eigen::Vector3d cur_pose(ego_x, ego_y, ego_theta);

  extand_trajectory();
  
  plan_interface::Gear gear = current_local_traj_.trajectory_points[0].gear;
  int v_gear_sign = (gear == plan_interface::Gear::D) ? 1 : -1;

  std::vector<double> s_vec;
  std::vector<double> t_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> v_vec;
  double mpc_ref_v = cfg_.target_vel * v_gear_sign;

  std::vector<Eigen::Vector3d> trajectory_b;
  for (auto &point : current_local_traj_.trajectory_points) {
    Eigen::Vector3d pose_w(point.x, point.y, utils::geometry::normalize_angle(point.yaw));
    Eigen::Vector3d pose_b = utils::geometry::transform_pose_2b(pose_w, cur_pose);
    trajectory_b.push_back(pose_b);
  }

  s_vec.push_back(0.0);
  t_vec.push_back(0.0);
  x_vec.push_back(trajectory_b.front().x());
  y_vec.push_back(trajectory_b.front().y());
  theta_vec.push_back(trajectory_b.front().z());
  v_vec.push_back(mpc_ref_v);
  // std::cout << "spline 0: s: " << s_vec.back() << " t: " << t_vec.back() <<
  //  " theta: " << theta_vec.back()*57.3 << " v: " << v_vec.back() << std::endl;
  for (int i = 1; i < trajectory_b.size(); ++i) {
    double ds = std::hypot(trajectory_b[i].x() - trajectory_b[i-1].x(),
                           trajectory_b[i-1].y() - trajectory_b[i].y());
    double dt = ds / std::abs(mpc_ref_v);
    s_vec.push_back(s_vec.back() + ds);
    t_vec.push_back(t_vec.back() + dt);
    x_vec.push_back(trajectory_b[i].x());
    y_vec.push_back(trajectory_b[i].y());
    theta_vec.push_back(trajectory_b[i].z());
    v_vec.push_back(mpc_ref_v);
  //   std::cout << "spline " << i << ": s: " << s_vec.back() << " t: " << t_vec.back() <<
  //  " theta: " << theta_vec.back()*57.3 << " v: " << v_vec.back() << std::endl;
  }
  utils::spline s_t(s_vec, t_vec);
  utils::spline s_x(s_vec, x_vec);
  utils::spline s_y(s_vec, y_vec);
  utils::spline s_theta(s_vec, theta_vec);
  utils::spline s_v(s_vec, v_vec);
  utils::spline t_theta(t_vec, theta_vec);

  double s_right = 0.0;
  double s_left = 0.0;
  double s_mid = 0.0;

  int start_index = get_start_index(cur_pos);

  s_right = s_vec[start_index];
  s_left = s_vec[start_index + 2 ];
  int iter_time = 10;
  for (int i = 0; i < iter_time; ++i) {
    s_mid = (s_left + s_right) / 2.0;
    double f_left = (Eigen::Vector2d(s_x(s_left), s_y(s_left))).dot(Eigen::Vector2d(s_x.deriv(1, s_left), s_y.deriv(1, s_left)));
    double f_mid = (Eigen::Vector2d(s_x(s_mid), s_y(s_mid))).dot(Eigen::Vector2d(s_x.deriv(1, s_mid), s_y.deriv(1, s_mid)));
    double f_right = (Eigen::Vector2d(s_x(s_right), s_y(s_right))).dot(Eigen::Vector2d(s_x.deriv(1, s_right), s_y.deriv(1, s_right)));
    if (f_left * f_mid < 0) {
      s_right = s_mid;
    } else {
      s_left = s_mid;
    }
  }
  double project_s = s_mid;
  remain_s_ = s_vec.back() - project_s - EXTAND_NUM * EXTAND_STEP;
  robot_model_->set_remain_s(remain_s_);

  robot_model_->set_lat_error(s_y(project_s));
  robot_model_->set_heading_error(s_theta(project_s));

  mpc_input.init_state_.x = 0.0;
  mpc_input.init_state_.y = 0.0;
  mpc_input.init_state_.theta = 0.0;
  mpc_input.init_state_.v = control_command_->linear.x;
  mpc_input.init_state_.omega = control_command_->angular.z;

  std::shared_ptr<plan_interface::MPCTrajectory> ref_traj = std::make_shared<plan_interface::MPCTrajectory>();

  for (int i = 0; i < cfg_.mpc_horizon + 1; ++i) {
    double s = project_s + i * cfg_.mpc_time_step * std::abs(mpc_ref_v);
    double x = s_x(s);
    double y = s_y(s);
    double theta = s_theta(s);
    double v = s_v(s);
    double omega = t_theta.deriv(1, s_t(s));
    Eigen::Vector3d pose_b(x, y, theta);
    mpc_input.x_ref_vec_.push_back(pose_b(0));
    mpc_input.y_ref_vec_.push_back(pose_b(1));
    mpc_input.theta_ref_vec_.push_back(pose_b(2));
    mpc_input.v_ref_vec_.push_back(v);
    mpc_input.omega_ref_vec_.push_back(omega);

    plan_interface::MPCState traj_point;
    Eigen::Vector3d pose_w = utils::geometry::transform_pose_2w(pose_b, cur_pose);
    traj_point.x = pose_w(0);
    traj_point.y = pose_w(1);
    traj_point.theta = pose_w(2);
    traj_point.v = v;
    traj_point.omega = omega;
    ref_traj->trajectory_points.push_back(traj_point);
  }
  ref_trajectory_ = ref_traj;
}

void MPCDiffAdapter::extand_trajectory() {
  plan_interface::TrajectoryPoint back_point = current_local_traj_.trajectory_points.back();
  double extend_dirction = ((back_point.gear == plan_interface::Gear::D) ? 0 : M_PI) + back_point.yaw;
  for (int i = 0; i < EXTAND_NUM; ++i) {
    plan_interface::TrajectoryPoint traj_point;
    traj_point.x = current_local_traj_.trajectory_points.back().x + EXTAND_STEP * std::cos(extend_dirction);
    traj_point.y = current_local_traj_.trajectory_points.back().y + EXTAND_STEP * std::sin(extend_dirction);
    traj_point.yaw = back_point.yaw;
    current_local_traj_.trajectory_points.push_back(traj_point);
  }
}

int MPCDiffAdapter::get_start_index(Eigen::Vector2d &cur_pos) {
  int start_index = utils::geometry::get_closest_traj_point_index(cur_pos, current_local_traj_);
  return std::max(0, start_index - 1);
}

void MPCDiffAdapter::apply_limits(const double v_cmd, const double omega_cmd) {
  const double max_v = robot_model_->params_.dyn.max_speed;
  const double max_omega = robot_model_->params_.dyn.max_yaw_rate;
  const double max_accel = robot_model_->params_.dyn.max_accel;
  const double max_decel = robot_model_->params_.dyn.max_decel;
  const double max_yaw_accel = robot_model_->params_.dyn.max_yaw_accel;

  double last_v_cmd = control_command_->linear.x;
  double last_omega_cmd = control_command_->angular.z;
  double delta_v = v_cmd - last_v_cmd;
  double delta_omega = omega_cmd - last_omega_cmd;

  double v = last_v_cmd + utils::geometry::clamp(delta_v, -max_decel / cfg_.fs, max_accel / cfg_.fs);
  double omega = last_omega_cmd + utils::geometry::clamp(delta_omega, -max_yaw_accel / cfg_.fs, max_yaw_accel / cfg_.fs);

  v = utils::geometry::clamp(v, -max_v, max_v);
  omega = utils::geometry::clamp(omega, -max_omega, max_omega);

  control_command_->linear.x = v;
  control_command_->angular.z = omega;
}

} // namespace actuator_adapter