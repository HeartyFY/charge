#include <cmath>
#include "geometry.h"
#include "pid_vel_controller.h"

namespace actuator_adapter
{

PIDController::PIDController(const Config& config) : cfg_(config) { reset(); }

double PIDController::compute(double target_value, double measured_value) {
  double error = target_value - measured_value;
  integral_ += error * cfg_.dt;
  double derivative = (error - prev_error_) / cfg_.dt;
  prev_error_ = error;
  return cfg_.kp * error + cfg_.ki * integral_ + cfg_.kd * derivative;
}

double PIDController::update(Input& input) {
  double v_ref = velocity_planner(input.remain_s);
  
  double v_cmd = compute(v_ref, input.v_fb);
  v_cmd = apply_limits(v_cmd, input.v_fb, input.remain_s);
  
  last_v_cmd_ = v_cmd;
  return v_cmd;
}

void PIDController::reset() {
  prev_error_ = 0.0;
  integral_ = 0.0;
  last_v_cmd_ = 0.0;
}

double PIDController::apply_limits(double v_cmd, double v_fb, double remain_s) {
  const double delta_v = v_cmd - last_v_cmd_;
  const double max_delta = (delta_v >= 0) ? (cfg_.max_accel * cfg_.dt) : (cfg_.max_decel * cfg_.dt);

  v_cmd = last_v_cmd_ + utils::geometry::clamp(delta_v, -cfg_.max_decel * cfg_.dt, cfg_.max_accel * cfg_.dt);

  v_cmd = utils::geometry::clamp(v_cmd, -cfg_.max_speed, cfg_.max_speed);

  if (remain_s < 0.1) {
    // const double emergency_decel = std::min(cfg_.max_decel, (v_fb * v_fb) / (2 * remain_s));
    // v_cmd = std::max(0.0, v_fb - emergency_decel * cfg_.dt);
    v_cmd = 0.0;
  }

  return v_cmd;
}

double PIDController::velocity_planner(double remain_s) {
  remain_s = std::max(remain_s, 0.0);

  const double target_speed = cfg_.max_speed;
  const double brake_distance = (target_speed * target_speed) / (2 * cfg_.max_decel/4);

  if (remain_s <= brake_distance) {
    // v² = 2*a*s
    return std::min(target_speed, std::sqrt(2 * cfg_.max_decel * remain_s));
  }
  return target_speed;
}

} // namespace actuator_adapter