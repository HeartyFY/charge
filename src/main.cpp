#include "cmake_config.h"
#include "nav_planner.h"

int main(int argc, char** argv) {

  NavPlannerConfig config;
  config.topic_config_file = std::string(PROJ_DIR) + "/config/topic_config.yaml";
  config.robot_params_file = std::string(PROJ_DIR) + "/config/robot_params.json";
  config.planner_config_file = std::string(PROJ_DIR) + "/config/planner_config.json";

  ros::init(argc, argv, "nav_planner_node");
  ros::NodeHandle nh;

  std::unique_ptr<NavPlanner> planner = std::make_unique<NavPlanner>(&nh, config);

  if (planner != nullptr) {
    planner->start();
    std::cout << "Planner started" << std::endl;
  } else {
    std::cout << "Planner failed to start" << std::endl;
    return -1;
  }

  ros::spin();

  return 0;
}