#ifndef TEB_PLANNER
#define TEB_PLANNER

#include "robot_model.h"
#include "local_planner.h"

namespace local_planner 
{

class TEBPlanner : public LocalPlanner {
public:
  TEBPlanner(std::shared_ptr<RobotModel> robot_model);
  ~TEBPlanner() override = default;

  bool update() override;
  std::shared_ptr<plan_interface::Trajectory> get_result() override;
  void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, 
                const ObstacleMsg& obs, const plan_interface::Path& glob_path) override;
  
private:
  double optimize(std::shared_ptr<plan_interface::Trajectory> init_traj, double learning_rate);
  double compute_tangent_yaw(std::shared_ptr<plan_interface::Trajectory> traj, size_t index);
  void construct_traj(std::shared_ptr<plan_interface::Trajectory> opt_traj);
  bool is_collision(double x, double y, double yaw);
  
  
private:
  std::shared_ptr<RobotModel> robot_model_;
  MapInfoMsg current_map_;
  OdomInfoMsg current_odom_;
  ObstacleMsg current_obs_;
  plan_interface::Path glob_path_;

  std::shared_ptr<plan_interface::Trajectory> planned_trajectory_;
}; // class TEBPlanner

} // namespace local_planner

#endif // TEB_PLANNER