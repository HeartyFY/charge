#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <queue>
#include <unordered_set>
#include <functional>

#include "robot_model.h"
#include "global_planner.h"

#include <visualization_msgs/MarkerArray.h>

namespace global_planner 
{

struct Node {
  double cost; 
  double g;
  double h;
  int x, y;
  Node* parent;

  Node(int x_, int y_, double g_, double h_, Node* parent_ = nullptr)
      : x(x_), y(y_), g(g_), h(h_), cost(g_ + h_), parent(parent_) {}
};

struct CompareNode {
  bool operator()(const Node& a, const Node& b) const {
    return a.cost > b.cost; // 较小的 cost 优先
  }
};

class AstarPlanner : public GlobalPlanner {
public:
  struct Config {
    int fs = 10;

    double front_expansion = 0.15;

    double bounding_radius;
  };

  AstarPlanner(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & global_config_json);
  ~AstarPlanner() override = default;

  bool update() override;
  std::shared_ptr<plan_interface::Path> get_result() override;
  void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, const ObstacleMsg& obs,
                const navi_types::Waypoint& goal, const plan_interface::Gear& gear) override;
  visualization_msgs::MarkerArray get_collision_mks() override;
private:
  void parse_config_json(const nlohmann::json & global_config_json);
  bool is_collision(int gx, int gy);
  double heuristic(int x1, int y1, int x2, int y2);
  std::shared_ptr<plan_interface::Path> reconstruct_path(Node* goal_node);

  void init_mk();

private:
  bool is_glob_collision(int gx, int gy);
  bool is_loc_collision(int lx, int ly);

private:
  Config cfg_;
  std::shared_ptr<RobotModel> robot_model_;
  MapInfoMsg current_map_;
  OdomInfoMsg current_odom_;
  ObstacleMsg current_obs_;
  navi_types::Waypoint current_goal_;

  std::shared_ptr<plan_interface::Path> planned_path_;
  Eigen::Vector3d glob_map_origin_, loc_map_origin_;

  visualization_msgs::MarkerArray collision_mks;
  mutable visualization_msgs::Marker global_collision_mk;
  mutable visualization_msgs::Marker local_collision_mk;
}; // class AstarPlanner

} // namespace global_planner

#endif // ASTAR_PLANNER_H
