#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "thread_base.h"
#include "plan_interface.h"
#include <visualization_msgs/MarkerArray.h>

namespace global_planner 
{
  
class GlobalPlanner : public ThreadBase {

public:
  virtual ~GlobalPlanner() = default;
  void run();
  void stop();

  void call_update();
  plan_interface::GlobalPlannerState get_state();
  
  virtual bool update() = 0;
  virtual std::shared_ptr<plan_interface::Path> get_result() = 0;
  virtual void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, const ObstacleMsg& obs,
                        const navi_types::Waypoint& goal,  const plan_interface::Gear& gear) = 0;
  virtual visualization_msgs::MarkerArray get_collision_mks() = 0;

  inline void set_charge_state(const bool is_charge) { charge_state_ = is_charge; }
  inline bool get_charge_state() const { return charge_state_; }

protected:
  void Process() override;

private:
  void run_once();

private:
  bool is_running_ = false;
  bool charge_state_{false};
  DoubleBuffer<plan_interface::GlobalPlannerState> state_bf_;

}; // class GlobalPlanner

} // namespace global_planner

#endif // GLOBAL_PLANNER_H