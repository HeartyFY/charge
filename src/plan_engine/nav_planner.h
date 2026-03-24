#ifndef NAV_PLANNER_H
#define NAV_PLANNER_H

#include <string>
#include <memory>

#include "thread_base.h"
#include "msg_handle.h"
#include "robot_model.h"
#include "planning_task.h"
#include "plan_interface.h"

struct NavPlannerConfig {
  std::string topic_config_file;
  std::string robot_params_file;
  std::string planner_config_file;
};

class NavPlanner : public ThreadBase {
public:
  NavPlanner(ros::NodeHandle* nh, const NavPlannerConfig& config);
  ~NavPlanner();

  void Process() override;
  void on_running();

  void start();
  void stop();

private:
  bool run_once(uint64_t start_time_ms);
  void register_msg_callback();

private:
  std::shared_ptr<MsgHandle> msg_handle_;
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<PlanningTask> planning_task_;
  std::shared_ptr<plan_interface::DataSource> data_source_;
  bool is_running_ = false;

};


#endif // NAV_PLANNER_H