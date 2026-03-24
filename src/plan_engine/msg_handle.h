#ifndef MSG_HANDLE_H
#define MSG_HANDLE_H

#include <map>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "ros_msg.h"

class MsgHandle {
public:
  MsgHandle(ros::NodeHandle* nh, const std::string &topic_config_file);
  ~MsgHandle();

  void set_map_info_callback(MapInfoCallback callback);
  void set_odom_info_callback(OdomInfoCallback callback);
  void set_obstacle_callback(ObstacleCallback callback);
  void set_task_info_callback(TaskInfoCallback callback);
  void set_joystick_callback(JoystickCallback callback);

  void publish_control_command(const ControlCommandMsg& control_command);
  void publish_plan_debug_info(const PlanDebugInfoMsg& plan_debug_info);
  void publish_navigation_status(const NavigationStatusMsg& navigation_status);

private:
  void load_topic(const std::string& topic_config_file);
  void init_publisher();

private:
  ros::NodeHandle* nh_;
  std::map<std::string, std::string> input_topic_map_;
  std::map<std::string, std::string> output_topic_map_;

  std::shared_ptr<ros::Subscriber> map_info_sub_;
  std::shared_ptr<ros::Subscriber> odom_info_sub_;
  std::shared_ptr<ros::Subscriber> obstacle_sub_;
  std::shared_ptr<ros::Subscriber> task_info_sub_;
  std::shared_ptr<ros::Subscriber> joystick_sub_;

  std::shared_ptr<ros::Publisher> control_command_pub_;
  std::shared_ptr<ros::Publisher> plan_debug_info_pub_;
  std::shared_ptr<ros::Publisher> navigation_status_pub_;
};


#endif // MSG_HANDLE_H