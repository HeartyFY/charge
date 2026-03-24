#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "plan_interface.h"

namespace local_planner
{

class LocalPlanner {
public:
  virtual ~LocalPlanner() = default;

  virtual bool update() = 0;
  virtual std::shared_ptr<plan_interface::Trajectory> get_result() = 0;
  virtual void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, const ObstacleMsg& obs, const plan_interface::Path& glob_path) = 0;
  
}; // class LocalPlanner

} // namespace local_planner


#endif // LOCAL_PLANNER_H