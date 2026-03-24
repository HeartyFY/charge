#include "mpc_solver.h" 
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

namespace actuator_adapter
{

constexpr auto NX  = ACADO_NX;  /* Number of differential state variables.  */
constexpr auto NU  = ACADO_NU;  /* Number of control inputs. */
constexpr auto NOD = ACADO_NOD; /* Number of online data values. */
constexpr auto NY  = ACADO_NY;  /* Number of measurements/references on nodes 0..N - 1. */
constexpr auto NYN = ACADO_NYN; /* Number of measurements/references on node N. */
constexpr auto N   = ACADO_N;   /* Number of intervals in the horizon. */

MPCSolver::MPCSolver() {
  init();
}

void MPCSolver::init() {
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));
	acado_initializeSolver();
}

bool MPCSolver::update(MPCInput& mpc_input) {
  MpcState ret = SOLVE_FAIL;
  ret = solve(mpc_input);

  if (ret == SOLVE_SUCCESS) {
    return true;
  } else {
    return false;
  }
}

MpcState MPCSolver::solve(MPCInput& mpc_input) {
  if (mpc_input.x_ref_vec_.size() != N + 1 || 
      mpc_input.y_ref_vec_.size() != N + 1 ||
      mpc_input.theta_ref_vec_.size() != N + 1 || 
      mpc_input.v_ref_vec_.size() != N + 1 ||
      mpc_input.omega_ref_vec_.size() != N + 1) {
    mpc_output_.update_state_ = INPUT_SIZE_FAULT;
    std::cout << "MPC input size fault!" << std::endl;
    return INPUT_SIZE_FAULT;
  }

  // Prepare references
	for (int i = 0; i < N; ++i) {
		acadoVariables.y[i * NY + 0] = mpc_input.x_ref_vec_[i];
		acadoVariables.y[i * NY + 1] = mpc_input.y_ref_vec_[i];
		acadoVariables.y[i * NY + 2] = mpc_input.theta_ref_vec_[i];
		acadoVariables.y[i * NY + 3] = mpc_input.v_ref_vec_[i];
		acadoVariables.y[i * NY + 4] = mpc_input.omega_ref_vec_[i];
		acadoVariables.y[i * NY + 5] = 0; // a
		acadoVariables.y[i * NY + 6] = 0; // alpha
	}

	acadoVariables.yN[ 0 ] = mpc_input.x_ref_vec_[ N ];
	acadoVariables.yN[ 1 ] = mpc_input.y_ref_vec_[ N ];
	acadoVariables.yN[ 2 ] = mpc_input.theta_ref_vec_[ N ];
	acadoVariables.yN[ 3 ] = mpc_input.v_ref_vec_[ N ];
	acadoVariables.yN[ 4 ] = mpc_input.omega_ref_vec_[ N ];

  // Current state feedback
  acadoVariables.x0[0] = mpc_input.init_state_.x;
  acadoVariables.x0[1] = mpc_input.init_state_.y;
  acadoVariables.x0[2] = mpc_input.init_state_.theta;
  acadoVariables.x0[3] = mpc_input.init_state_.v;
  acadoVariables.x0[4] = mpc_input.init_state_.omega;

  // set weights
  for (int i = 0; i < N; i++) {
		acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weights_.q_x;
		acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weights_.q_y;
		acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weights_.q_theta;
		acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weights_.q_v;
    acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weights_.q_omega;
    acadoVariables.W[NY * NY * i + (NY + 1) * 5] = weights_.r_a;
    acadoVariables.W[NY * NY * i + (NY + 1) * 6] = weights_.r_alpha;
	}
	acadoVariables.WN[(NYN + 1) * 0] = weights_.q_x_n;
	acadoVariables.WN[(NYN + 1) * 1] = weights_.q_y_n;
	acadoVariables.WN[(NYN + 1) * 2] = weights_.q_theta_n;
	acadoVariables.WN[(NYN + 1) * 3] = weights_.q_v_n;
  acadoVariables.WN[(NYN + 1) * 4] = weights_.q_omega_n;

  // solve the problem
  acado_timer t;
  auto ret_pre = acado_preparationStep();
  acado_tic(&t);
  auto ret_fb = acado_feedbackStep();
  real_t te = acado_toc(&t);

  if ((!ret_pre) && (!ret_fb)) {
    // std::cout << "MPC solver SOLVE_SUCCEED " << std::endl;
    mpc_output_.update_state_ = SOLVE_SUCCESS;
  } else {
    mpc_output_.update_state_ = SOLVE_FAIL;
    std::cout << "MPC solver SOLVE_FAIL " << std::endl;
    return SOLVE_FAIL;
  }

  // get solution
  clear_mpc_output();

  bool contains_nan = false;
  for (int i = 0; i < N; ++i){
    for (int j = 0; j < NU; ++j) {
      if (std::isnan(acadoVariables.u[i * NU + j]) || 
          std::isinf(acadoVariables.u[i * NU + j])) {
        contains_nan = true;
      }
    }
    for (int j = 0; j < NX; ++j) {
      if (std::isnan(acadoVariables.x[i * NX + j]) || 
          std::isinf(acadoVariables.x[i * NX + j])) {
        contains_nan = true;
      }
    }
  }

  if (contains_nan) {
    auto out = acado_initializeSolver();
    mpc_output_.update_state_ = SOLVE_FAIL;
    std::cout << "MPC solver SOLVE_FAIL contains_nan" << std::endl;
    return SOLVE_FAIL;
  }

  // get state： size = N + 1
  for (int i = 0; i < N + 1; ++i) {
    mpc_output_.t_mpc_vec_.push_back(static_cast<double>(i) * 0.2);
    mpc_output_.x_mpc_vec_.push_back(acadoVariables.x[i * NX]);
    mpc_output_.y_mpc_vec_.push_back(acadoVariables.x[i * NX + 1]);
    mpc_output_.theta_mpc_vec_.push_back(acadoVariables.x[i * NX + 2]);
    mpc_output_.v_mpc_vec_.push_back(acadoVariables.x[i * NX + 3]);
    mpc_output_.omega_mpc_vec_.push_back(acadoVariables.x[i * NX + 4]);
  }

  // get input: size = N
  for (int i = 0; i < N; ++i) {
    mpc_output_.a_mpc_vec_.push_back(acadoVariables.u[i * NU]);
    mpc_output_.alpha_mpc_vec_.push_back(acadoVariables.u[i * NU + 1]);
  }

  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  mpc_output_.iteration_ = acado_getNWSR();
  mpc_output_.update_state_ = SOLVE_SUCCESS;
  // std::cout << "SOLVE_SUCCESS iteration: " << mpc_output_.iteration_ << std::endl;
  return SOLVE_SUCCESS;
}

void MPCSolver::clear_mpc_output() {
  mpc_output_.t_mpc_vec_.reserve(N + 1);
  mpc_output_.t_mpc_vec_.clear();

  mpc_output_.x_mpc_vec_.reserve(N + 1);
  mpc_output_.x_mpc_vec_.clear();
  mpc_output_.y_mpc_vec_.reserve(N + 1);
  mpc_output_.y_mpc_vec_.clear();
  mpc_output_.theta_mpc_vec_.reserve(N + 1);
  mpc_output_.theta_mpc_vec_.clear();
  mpc_output_.v_mpc_vec_.reserve(N + 1);
  mpc_output_.v_mpc_vec_.clear();
  mpc_output_.omega_mpc_vec_.reserve(N + 1);
  mpc_output_.omega_mpc_vec_.clear();

  mpc_output_.a_mpc_vec_.reserve(N);
  mpc_output_.a_mpc_vec_.clear();
  mpc_output_.alpha_mpc_vec_.reserve(N);
  mpc_output_.alpha_mpc_vec_.clear();
}

void MPCSolver::reset() {
  init();
}

} // namespace actuator_adapter
