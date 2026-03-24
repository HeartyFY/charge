#ifndef HYBIRD_ASTAR_H
#define HYBIRD_ASTAR_H

#include <queue>
#include <unordered_map>

#include "kdtree.h"
#include "grid_map.h"
#include "robot_model.h"
#include "global_planner.h"

// #include 

namespace global_planner {

namespace hybird_astar{

class HolonomicHeuristic {
public:
  struct Node {
    Eigen::Vector2i coord_;
    int index_;
    double g_;
    std::shared_ptr<Node> parent_;

    Node(const Eigen::Vector2i& coord) : coord_(coord), g_(0), parent_(nullptr) {}
  };

  struct CompareNode {
    bool operator() (const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
      return a->g_ > b->g_;
    }
  };
  
  HolonomicHeuristic(std::shared_ptr<RobotModel> robot_model, std::shared_ptr<GridMap> grid, 
                    std::shared_ptr<utils::kdtree::KDTree2D> obst_kdtree, double safe_buffer);
  ~HolonomicHeuristic() = default;
  void gen_heur_map(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& goal_pos);
  double get_heur(const Eigen::Vector2d& pos);

  visualization_msgs::Marker get_collision_mk();

private:
  std::vector<std::shared_ptr<Node>> get_neighbors(const std::shared_ptr<Node>& node);
  bool is_valid(const std::shared_ptr<Node>& node);
  bool is_collision(const std::shared_ptr<Node>& node);
  bool is_collision(const Eigen::Vector2d& pos);
  void init_mk();

private:
  double safe_buffer_;
  std::shared_ptr<GridMap> grid_;
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<utils::kdtree::KDTree2D> obst_kdtree_;

  std::unordered_map<int, std::shared_ptr<Node>> heuristic_map_;

  visualization_msgs::Marker collision_mk;
}; // class HolonomicHeuristic

class NonHolonomicHeuristic {

}; // class NonHolonomicHeuristic



class HybirdAstar : public GlobalPlanner {
public:
  struct Config {
    double reverse_penalty;
    double rotation_penalty;
    double gear_switch_penalty;
    double steering_penalty;
    double steering_switch_penalty;

    double arc_length;
    double arc_step_length;
    double end_point_line_length;

    double grid_resolution;
    double angle_resolution;
    double grid_radius_offset;

    double heuristic_safe_buffer;
    double search_safe_buffer;
    double charge_search_safe_buffer;

    double max_plan_time_ms;
  };

  struct Node {
    Eigen::Vector3d pose_;
    std::vector<Eigen::Vector3d> path_;
    int index_;
    double g_, h_;
    std::shared_ptr<Node> parent_;
    plan_interface::Gear plan_gear_;
    int steering_;
    Node(const Eigen::Vector3d& pose) : pose_(pose), index_(0), g_(0.0), h_(0.0), parent_(nullptr) {
      path_.push_back(pose);
    }
    Node(const std::vector<Eigen::Vector3d>& path_) 
      : pose_(path_.back()), index_(0), g_(0.0), h_(0.0), parent_(nullptr), path_(path_) {}
    
    double get_cost() { return g_ + h_; }
  };

  struct Compare {
    bool operator() (const std::pair<int, double>& a, const std::pair<int, double>& b) const {
      return a.second > b.second;
    }
  };

  HybirdAstar(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & global_config_json);
  ~HybirdAstar() override = default;

  bool update() override;
  std::shared_ptr<plan_interface::Path> get_result() override;
  void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, const ObstacleMsg& obs,
                const navi_types::Waypoint& goal, const plan_interface::Gear& gear) override;
  visualization_msgs::MarkerArray get_collision_mks() override;


private:
  bool set_start_and_goal();
  void searching();
  void init_mk();
  void init_kdtree();
  void set_result();
  void set_grid();
  void get_charge_path();
  void calculate_cost(const std::shared_ptr<Node>& node);
  std::vector<std::shared_ptr<Node>> gen_neighbors(const std::shared_ptr<Node>& node);
  bool analytic_expansion(const std::shared_ptr<Node>& node);
  bool is_collision(const Eigen::Vector3d& pose);
  bool is_collision(const std::vector<Eigen::Vector3d>& path);
  void parse_config_json(const nlohmann::json & global_config_json);
  std::vector<Eigen::Vector3d> get_arc_traj(const Eigen::Vector3d& pose_0, const std::string& direction);
  std::vector<Eigen::Vector3d> get_trans_traj(const Eigen::Vector3d& pose_0, const Eigen::Vector3d& pose_f);


private:
  Config cfg_;
  std::shared_ptr<RobotModel> robot_model_;
  MapInfoMsg current_map_;
  OdomInfoMsg current_odom_;
  ObstacleMsg current_obs_;
  navi_types::Waypoint current_goal_;
  navi_types::Waypoint charge_goal_;
  plan_interface::Gear current_gear_;

  std::shared_ptr<GridMap> grid_;
  std::shared_ptr<utils::kdtree::KDTree2D> obst_kdtree_;
  std::shared_ptr<plan_interface::Path> planned_path_;

  std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, Compare> open_pq_;
  std::unordered_map<int, std::shared_ptr<Node>> open_set_, close_set_;
  std::shared_ptr<Node> start_node_, goal_node_, end_node_;
  std::shared_ptr<HolonomicHeuristic> hol_heur_;

  std::vector<std::string> directions_;
  std::vector<std::string> forward_directions_{"RF", "RFF", "F", "LFF", "LF"};
  std::vector<std::string> reverse_directions_{"RB", "RBB", "B", "LBB", "LB"};
  std::vector<std::string> rotate_directions_{"RRF", "LLF"};

  visualization_msgs::MarkerArray collision_mks;
  visualization_msgs::Marker expand_path_mk;
  visualization_msgs::Marker second_path_mk;

}; // class HybireAstar


} // namespace hybird_astar

} // namespace global_planner

#endif // HYBIRD_ASTAR_H