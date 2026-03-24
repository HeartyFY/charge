#ifndef ACTUATOR_ADAPTER_H
#define ACTUATOR_ADAPTER_H

#include "plan_interface.h"

namespace actuator_adapter
{

class ActuatorAdapter {
public:
  virtual ~ActuatorAdapter() = default;

  virtual bool update() = 0;
  virtual std::shared_ptr<ControlCommandMsg> get_result() = 0;
  virtual std::shared_ptr<plan_interface::MPCTrajectory> get_mpc_trajectory() = 0;
  virtual std::shared_ptr<plan_interface::MPCTrajectory> get_ref_trajectory() = 0;
  virtual void feed_data(const OdomInfoMsg& odom, const plan_interface::Trajectory& local_traj) = 0;
  virtual void reset() = 0;

}; // class ActuatorAdapter

} // namespace actuator_adapter


#endif // ACTUATOR_ADAPTER_H