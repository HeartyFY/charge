#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include "plan_interface.h"

namespace actuator_adapter
{

enum MpcState {
  SOLVE_SUCCESS = 0,
  ERROR_LARGE = 1,
  SOLVE_FAIL = 2,
  INPUT_SIZE_FAULT = 3,
};

struct Weights {
  double q_x;
  double q_y;
  double q_theta;
  double q_v;
  double q_omega;

  double q_x_n;
  double q_y_n;
  double q_theta_n;
  double q_v_n;
  double q_omega_n;

  double r_a;
  double r_alpha;

  Weights() : q_x(0.0), q_y(0.0), q_theta(0.0), q_v(0.0), q_omega(0.0), 
              q_x_n(0.0), q_y_n(0.0), q_theta_n(0.0), q_v_n(0.0), q_omega_n(0.0),
              r_a(0.0), r_alpha(0.0) {}
};

struct Bounds {
  double v_lower_bound;
  double v_upper_bound;
  double omega_lower_bound;
  double omega_upper_bound;
  double a_lower_bound;
  double a_upper_bound;
  double alpha_lower_bound;
  double alpha_upper_bound;
};

struct State {
  double x;
  double y;
  double theta;
  double v;
  double omega;
};

struct Ctrl {
  double a;
  double alpha;
};


struct MPCInput {
  State init_state_;

  std::vector<double> x_ref_vec_;
  std::vector<double> y_ref_vec_;
  std::vector<double> theta_ref_vec_;
  std::vector<double> v_ref_vec_;
  std::vector<double> omega_ref_vec_;
};

struct MPCOutput {
  std::vector<double> t_mpc_vec_;
  std::vector<double> x_mpc_vec_;
  std::vector<double> y_mpc_vec_;
  std::vector<double> theta_mpc_vec_;
  std::vector<double> v_mpc_vec_;
  std::vector<double> omega_mpc_vec_;
  std::vector<double> a_mpc_vec_;
  std::vector<double> alpha_mpc_vec_;

  int iteration_;
  MpcState update_state_;
};

class MPCSolver {
public:
  MPCSolver();
  void init();
  bool update(MPCInput& mpc_input);
  void reset();
  MPCOutput get_output() { return mpc_output_; };
  void set_bounds(const Bounds& bounds) { bounds_ = bounds; };
  void set_weights(const Weights& weights) { weights_ = weights; };
private:
  MpcState solve(MPCInput& mpc_input);
  void clear_mpc_output();

private:
  // bool init_flag_{false};
  Bounds bounds_;
  Weights weights_;
  MPCOutput mpc_output_;
};

} // namespace actuator_adapter

#endif // MPC_SOLVER_H