#include "lbfgs_optimizer.h"
#include "utils.h"
#include "geometry.h"

namespace local_planner {
namespace lbfgs_optimizer {

LbfgsOptimizer::LbfgsOptimizer(std::shared_ptr<RobotModel> robot_model, const nlohmann::json & local_config_json)
 : robot_model_(robot_model){
  planned_trajectory_ = std::make_shared<plan_interface::Trajectory>();
  obst_kdtree_ = std::make_unique<utils::kdtree::KDTree2D>();
  parse_config_json(local_config_json);
}

void LbfgsOptimizer::parse_config_json(const nlohmann::json & local_config_json) {
  cfg_.safe_distance = local_config_json.contains("safe_distance") ? local_config_json.at("safe_distance").get<double>() : 0.8;
  cfg_.max_kappa = local_config_json.contains("max_kappa") ? local_config_json.at("max_kappa").get<double>() : 1/4.0;
  cfg_.smooth_weight = local_config_json.contains("smooth_weight") ? local_config_json.at("smooth_weight").get<double>() : 100.0;
  cfg_.curvature_weight = local_config_json.contains("curvature_weight") ? local_config_json.at("curvature_weight").get<double>() : 0.0;
  cfg_.obstacle_weight = local_config_json.contains("obstacle_weight") ? local_config_json.at("obstacle_weight").get<double>() : 20.0;

  cfg_.lbfgs_mem_size = local_config_json.contains("lbfgs_mem_size") ? local_config_json.at("lbfgs_mem_size").get<int>() : 8;
  cfg_.lbfgs_past = local_config_json.contains("lbfgs_past") ? local_config_json.at("lbfgs_past").get<int>() : 3;
  cfg_.lbfgs_min_step = local_config_json.contains("lbfgs_min_step") ? local_config_json.at("lbfgs_min_step").get<double>() : 1.0e-32;
  cfg_.lbfgs_g_epsilon = local_config_json.contains("lbfgs_g_epsilon") ? local_config_json.at("lbfgs_g_epsilon").get<double>() : 0.0;
  cfg_.lbfgs_delta = local_config_json.contains("lbfgs_delta") ? local_config_json.at("lbfgs_delta").get<double>() : 5.0e-4;
  cfg_.lbfgs_max_iterations = local_config_json.contains("lbfgs_max_iterations") ? local_config_json.at("lbfgs_max_iterations").get<int>() : 1000;
  

  std::cout << "LBFGS Optimizer initialized" << std::endl;
  std::cout << "cfg_.safe_distance: " << cfg_.safe_distance << std::endl;
  std::cout << "cfg_.max_kappa: " << cfg_.max_kappa << std::endl;
  std::cout << "cfg_.smooth_weight: " << cfg_.smooth_weight << std::endl;
  std::cout << "cfg_.curvature_weight: " << cfg_.curvature_weight << std::endl;
  std::cout << "cfg_.obstacle_weight: " << cfg_.obstacle_weight << std::endl;

  std::cout << "cfg_.lbfgs_mem_size: " << cfg_.lbfgs_mem_size << std::endl;
  std::cout << "cfg_.lbfgs_past: " << cfg_.lbfgs_past << std::endl;
  std::cout << "cfg_.lbfgs_min_step: " << cfg_.lbfgs_min_step << std::endl;
  std::cout << "cfg_.lbfgs_g_epsilon: " << cfg_.lbfgs_g_epsilon << std::endl;
  std::cout << "cfg_.lbfgs_delta: " << cfg_.lbfgs_delta << std::endl;
  std::cout << "cfg_.lbfgs_max_iterations: " << cfg_.lbfgs_max_iterations << std::endl;
}

void LbfgsOptimizer::feed_data(const MapInfoMsg &map, const OdomInfoMsg &odom,
                          const ObstacleMsg &obs, const plan_interface::Path &glob_path) {
  current_map_ = map;
  current_odom_ = odom;
  current_obs_ = obs;
  glob_path_ = glob_path;
  // init_kdtree();
}

std::shared_ptr<plan_interface::Trajectory> LbfgsOptimizer::get_result() {
  return planned_trajectory_;
}

bool LbfgsOptimizer::update() {
  if (glob_path_.path_points.size() < 3) {
    std::cerr << "Global path is empty. Cannot plan trajectory.\n";
    return false;
  }

  // auto trajectory = optimize();
  auto trajectory = interpolation();

  if (trajectory == nullptr || trajectory->trajectory_points.empty()) {
    std::cerr << "Trajectory optimization failed.\n";
    return false;
  }
  planned_trajectory_ = trajectory;
  planned_trajectory_->is_last_trajectory = glob_path_.is_last_path;

  return true;
}

std::shared_ptr<plan_interface::Trajectory> LbfgsOptimizer::optimize() {
  const int N = glob_path_.path_points.size();
  const int D = 3;
  Eigen::VectorXd x(D * N);

 
  for (int i = 0; i < N; ++i) {
    x[D * i] = glob_path_.path_points[i].x;
    x[D * i + 1] = glob_path_.path_points[i].y;
    x[D * i + 2] = glob_path_.path_points[i].yaw;
  }

  lbfgs::lbfgs_parameter_t params;
  params.mem_size = cfg_.lbfgs_mem_size;
  params.past = cfg_.lbfgs_past;
  params.min_step = cfg_.lbfgs_min_step;
  params.g_epsilon = cfg_.lbfgs_g_epsilon;
  params.delta = cfg_.lbfgs_delta;
  params.max_iterations = cfg_.lbfgs_max_iterations;
  

  double final_cost;
  uint64_t start_time_ms = utils::time_now_ms();
  int ret = lbfgs::lbfgs_optimize(x, final_cost, cost_function, NULL, NULL, this, params);
  std::cout << "ret: " << ret << "; cost: " << final_cost << "; use: " << utils::time_now_ms() - start_time_ms << "ms" << std::endl;

  std::shared_ptr<plan_interface::Trajectory> trajectory = std::make_shared<plan_interface::Trajectory>();
  // if (ret < 0) return trajectory;

  trajectory->trajectory_points.push_back(plan_interface::TrajectoryPoint(x[0], x[1], x[2]));
  trajectory->trajectory_points.push_back(plan_interface::TrajectoryPoint(x[D * 1], x[D * 1 + 1], x[D * 1 + 2]));
  for (int i = 2; i < N - 2; ++i) {
    plan_interface::TrajectoryPoint p;
    p.x = x[D * i];
    p.y = x[D * i + 1];
    p.yaw = std::atan2(x[D * (i+1) + 1] - p.y, x[D * (i+1)] - p.x);
    trajectory->trajectory_points.push_back(p);
  }
  trajectory->trajectory_points.push_back(plan_interface::TrajectoryPoint(x[D * (N-2)], x[D * (N-2) + 1], x[D * (N-2) + 2]));
  trajectory->trajectory_points.push_back(plan_interface::TrajectoryPoint(x[D * (N-1)], x[D * (N-1) + 1], x[D * (N-1) + 2]));
  trajectory->length = glob_path_.length;
  trajectory->segments = glob_path_.segments;
  trajectory->is_last_trajectory = glob_path_.is_last_path;

  return trajectory;
}

double LbfgsOptimizer::cost_function(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
  const int D = 3; // 每个点的维度 (x, y, yaw)
  const int N = x.size() / 3; // 总轨迹点数

  LbfgsOptimizer* opt = static_cast<LbfgsOptimizer*>(instance);

  g.setZero();
  double cost = 0.0;

  // 遍历中间点（首尾各两个点不进行优化）
  for (int i = 2; i < N - 2; ++i) {
    Eigen::Vector2d p(x[D * i], x[D * i + 1]);        // 当前点
    Eigen::Vector2d pp(x[D * (i - 1)], x[D * (i - 1) + 1]); // 前一个点
    Eigen::Vector2d ppp(x[D * (i - 2)], x[D * (i - 2) + 1]); // 前两个点
    Eigen::Vector2d pn(x[D * (i + 1)], x[D * (i + 1) + 1]);  // 后一个点
    Eigen::Vector2d pnn(x[D * (i + 2)], x[D * (i + 2) + 1]); // 后两个点

    // **Smooth Cost**
    Eigen::Vector2d smooth = pn - 2 * p + pp;
    double smooth_cost = opt->cfg_.smooth_weight * smooth.squaredNorm();
    cost += smooth_cost;
    g.segment<2>(D * i) += 2 * opt->cfg_.smooth_weight * (ppp - 4 * pp + 6 * p - 4 * pn + pnn);

    // **Collision Cost**
    Eigen::Vector2d nearest_obst = opt->obst_kdtree_->get_near(p);
    Eigen::Vector2d obst_to_p = p - nearest_obst;
    double dist = obst_to_p.norm();

    if (dist < opt->cfg_.safe_distance) {
      double collision_cost = opt->cfg_.obstacle_weight * std::pow(opt->cfg_.safe_distance - dist, 2);
      cost += collision_cost;
      g.segment<2>(D * i) += -2 * opt->cfg_.obstacle_weight * (opt->cfg_.safe_distance - dist) * obst_to_p / dist;
    }
    
  }

  return cost;
}

void LbfgsOptimizer::init_kdtree() {
  std::vector<Eigen::Vector2d> points;
  uint64_t start_time_ms = utils::time_now_ms();
  int glob_width = current_map_.info.width;
  int glob_height = current_map_.info.height;
  for (int x = 0; x < glob_width; x++) {
    for (int y = 0; y < glob_height; y++) {
      if (x == 0 || x == glob_width-1 || y == 0 || y == glob_height-1 || (current_map_.data[y * glob_width + x] > 0)) {
        auto [wx, wy] = robot_model_->glob_map_to_world_coords(x, y);
        points.push_back(Eigen::Vector2d(wx, wy));
      }
    }
  }

  if (!current_obs_.data.empty()) {
    int local_width = current_obs_.info.width;
    int local_height = current_obs_.info.height;

    for (int x = 0; x < local_width; x++) {
      for (int y = 0; y < local_height; y++) {
        if (current_obs_.data[y * local_width + x].occupancy) {
          auto [wx, wy] = robot_model_->loc_map_to_world_coords(x, y);
          points.push_back(Eigen::Vector2d(wx, wy));
        }
      }
    }
  }
  points.push_back(Eigen::Vector2d(-1000, -1000));
  
  obst_kdtree_->init(points);
  std::cout << "init_kdtree_use_ms: " << utils::time_now_ms() - start_time_ms << std::endl;

}

std::shared_ptr<plan_interface::Trajectory> LbfgsOptimizer::interpolation() {
  std::shared_ptr<plan_interface::Trajectory> trajectory = std::make_shared<plan_interface::Trajectory>();
  for (auto &p : glob_path_.path_points) {
    plan_interface::TrajectoryPoint tp;
    tp.x = p.x;
    tp.y = p.y;
    tp.yaw = p.yaw;
    tp.gear = p.gear;
    trajectory->trajectory_points.push_back(tp);
  }
  return trajectory;
}

  
} // namespace lbfgs_optimizer
} // namespace local_planner