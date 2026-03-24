#ifndef PID_VEL_CONTROLLER_H
#define PID_VEL_CONTROLLER_H

namespace actuator_adapter
{

class PIDController {
public:
  struct Config {
    double dt = 0.02;
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.01;
    double max_speed = 2.0;
    double max_accel = 1.0;
    double max_decel = 1.0; 
  };
  struct Input {
    double remain_s = 0.0;
    double v_fb = 0.0;
  };
  PIDController(const Config& config);
  double update(Input& input);
  void reset();
private:
  double compute(double target_value, double measured_value);
  double velocity_planner(double remain_s);
  double apply_limits(double v_cmd, double v_fb, double remain_s);

private:
  Config cfg_;
  double prev_error_;
  double integral_;
  double last_v_cmd_;
}; // class PIDController

} // namespace actuator_adapter

#endif // PID_VEL_CONTROLLER_H