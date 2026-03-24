#ifndef CTRL_SPACE_PLANNER
#define CTRL_SPACE_PLANNER
#include <set>
#include <Eigen/Dense>

#include "ros_msg.h"
#include "robot_model.h"
#include "local_planner.h"

namespace local_planner 
{
struct State {
  double x;
  double y;
  double yaw;
};

struct Ctrl {
  double v;
  double w;
};

struct StateNode {
  double cost; 
  double g;
  double h;
  int x, y;
  State state;
  Ctrl ctrl;
  plan_interface::Trajectory  segment;
  StateNode* parent;

  StateNode(int x_, int y_, double g_, double h_, StateNode* parent_ = nullptr)
      : x(x_), y(y_), g(g_), h(h_), cost(g_ + h_), parent(parent_) {}
};

struct CompareStateNode {
  bool operator()(const StateNode& a, const StateNode& b) const {
    return a.cost > b.cost; // 较小的 cost 优先
  }
};

class CtrlSpacePlanner : LocalPlanner {
public:
  CtrlSpacePlanner(std::shared_ptr<RobotModel> robot_model);
  ~CtrlSpacePlanner() override = default;

  // bool update() override;
  std::shared_ptr<plan_interface::Trajectory> get_result() override;
  void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, 
                const ObstacleMsg& obs, const plan_interface::Path& glob_path) override;

private:


private:
  std::shared_ptr<RobotModel> robot_model_;
  MapInfoMsg current_map_;
  OdomInfoMsg current_odom_;
  ObstacleMsg current_obs_;
  plan_interface::Path glob_path_;

  std::shared_ptr<plan_interface::Trajectory> planned_trajectory_;
}; // class CtrlSpacePlanner

} // namespace local_planner

#endif // CTRL_SPACE_PLANNER