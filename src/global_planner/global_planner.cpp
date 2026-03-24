#include "utils.h"
#include "global_planner.h"


namespace global_planner 
{

// GlobalPlanner::~GlobalPlanner() { stop(); }

void GlobalPlanner::run() {
  if (is_running_ = false) {
    is_running_ = true;
  }
  state_bf_.write(plan_interface::GlobalPlannerState::IDLE);
  start_thread();
}

void GlobalPlanner::stop() {
  if (is_running_ = true) {
    is_running_ = false;
  }
  stop_thread();
}

void GlobalPlanner::Process() {
  while (get_thread_state() == ThreadState::Running) {
    int64_t start_millis = utils::time_now_ms();
    run_once();
    int64_t end_millis = utils::time_now_ms();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void GlobalPlanner::call_update() {
  auto cur_state = state_bf_.read();
  if (cur_state.has_value() && cur_state.value() == plan_interface::GlobalPlannerState::IDLE) {
    state_bf_.write(plan_interface::GlobalPlannerState::PLANNING);
  }
}

void GlobalPlanner::run_once() {
  auto cur_state = state_bf_.read();
  if (cur_state.has_value() && cur_state.value() == plan_interface::GlobalPlannerState::PLANNING) {
    if (update()) {
      state_bf_.write(plan_interface::GlobalPlannerState::SUCCESS);
    } else {
      state_bf_.write(plan_interface::GlobalPlannerState::FAILED);
    }
  }
}

plan_interface::GlobalPlannerState GlobalPlanner::get_state() {
  auto cur_state = state_bf_.read();
  if (cur_state.has_value()) {
    switch (cur_state.value()) {
      case plan_interface::GlobalPlannerState::IDLE:
        return plan_interface::GlobalPlannerState::IDLE;
      case plan_interface::GlobalPlannerState::PLANNING:
        return plan_interface::GlobalPlannerState::PLANNING;
      case plan_interface::GlobalPlannerState::SUCCESS:
        state_bf_.write(plan_interface::GlobalPlannerState::IDLE);
        return plan_interface::GlobalPlannerState::SUCCESS;
      case plan_interface::GlobalPlannerState::FAILED:
        state_bf_.write(plan_interface::GlobalPlannerState::IDLE);
        return plan_interface::GlobalPlannerState::FAILED;
    }
  }
  state_bf_.write(plan_interface::GlobalPlannerState::IDLE);
  return plan_interface::GlobalPlannerState::IDLE;
}

} // namespace global_planner