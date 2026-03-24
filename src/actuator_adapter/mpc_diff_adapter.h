#ifndef MPC_DIFF_ADAPTER_H
#define MPC_DIFF_ADAPTER_H

#include "robot_model.h"
#include "actuator_adapter.h"
#include "mpc_solver.h"

namespace actuator_adapter
{

class MPCDiffAdapter : public ActuatorAdapter {
public:
  struct Config {
    int fs = 50;
    double target_vel = 0.3;
    int mpc_horizon = 20;
    double mpc_time_step = 0.2; // s
    int control_horizon_index = 1;
  };
  MPCDiffAdapter(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & actuator_config_reader);
  ~MPCDiffAdapter() override = default;

  bool update() override;
  std::shared_ptr<ControlCommandMsg> get_result() override;
  std::shared_ptr<plan_interface::MPCTrajectory> get_mpc_trajectory() override;
  std::shared_ptr<plan_interface::MPCTrajectory> get_ref_trajectory() override;
  void feed_data(const OdomInfoMsg& odom, const plan_interface::Trajectory& local_traj) override;
  void reset() override;
  
private:
  void parse_config_json(const nlohmann::json & actuator_config_reader);
  void pretreating(MPCInput &mpc_input);
  int get_start_index(Eigen::Vector2d &cur_pos);
  void set_mpc_bounds();
  void set_mpc_weights();
  void set_mpc_trajectory(const MPCOutput &mpc_output);
  void extand_trajectory();
  void apply_limits(const double v_cmd, const double omega_cmd);
  
private:
  Config cfg_;
  int EXTAND_NUM = 4;
  double EXTAND_STEP = 0.5;
  double remain_s_;
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<MPCSolver> mpc_solver_;
  OdomInfoMsg current_odom_;
  plan_interface::Trajectory current_local_traj_;

  std::shared_ptr<ControlCommandMsg> control_command_;
  std::shared_ptr<plan_interface::MPCTrajectory> mpc_trajectory_;
  std::shared_ptr<plan_interface::MPCTrajectory> ref_trajectory_;
}; // class MPCDiffAdapter

} // namespace actuator_adapter

#endif // MPC_DIFF_ADAPTER_H