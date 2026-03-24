#ifndef LBFGS_OPTIMIZER_H
#define LBFGS_OPTIMIZER_H


#include "lbfgs.hpp"
#include "kdtree.h"
#include "robot_model.h"
#include "local_planner.h"


namespace local_planner {
namespace lbfgs_optimizer {

class LbfgsOptimizer : public LocalPlanner {
public:
  struct Config {
    double safe_distance;
    double max_kappa;
    double smooth_weight;
    double curvature_weight;
    double obstacle_weight;

    int lbfgs_mem_size;
    int lbfgs_past;
    double lbfgs_min_step;
    double lbfgs_g_epsilon;
    double lbfgs_delta;
    int lbfgs_max_iterations;
  };

  LbfgsOptimizer(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & local_config_json);
  ~LbfgsOptimizer() override = default;

  bool update() override;
  std::shared_ptr<plan_interface::Trajectory> get_result() override;
  void feed_data(const MapInfoMsg& map, const OdomInfoMsg& odom, 
                const ObstacleMsg& obs, const plan_interface::Path& glob_path) override;

private:
  void parse_config_json(const nlohmann::json & local_config_json);
  void init_kdtree();
  std::shared_ptr<plan_interface::Trajectory> interpolation();
  std::shared_ptr<plan_interface::Trajectory> optimize();
  static double cost_function(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g);

private:
  Config cfg_;
  std::shared_ptr<RobotModel> robot_model_;
  MapInfoMsg current_map_;
  OdomInfoMsg current_odom_;
  ObstacleMsg current_obs_;
  plan_interface::Path glob_path_;

  std::shared_ptr<plan_interface::Trajectory> planned_trajectory_;

  std::unique_ptr<utils::kdtree::KDTree2D> obst_kdtree_;

}; // class LbfgsOptimizer
  
} // namespace lbfgs_optimizer

} // namespace local_planner

#endif // LBFGS_OPTIMIZER_H