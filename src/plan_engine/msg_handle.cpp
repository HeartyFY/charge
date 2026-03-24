#include "msg_handle.h"


MsgHandle::MsgHandle(ros::NodeHandle* nh, const std::string &topic_config_file) : nh_(nh) {
  load_topic(topic_config_file);
  init_publisher();
}

MsgHandle::~MsgHandle() { }

void MsgHandle::set_map_info_callback(MapInfoCallback callback) {
  std::string topic_name;
  try {
    topic_name = input_topic_map_.at("map_info");
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("map_info tpoic not found in topic config file" + std::string(e.what()));
  }
  map_info_sub_ = std::make_shared<ros::Subscriber>(nh_->subscribe<MapInfoMsg>(topic_name, 10, callback));
}

void MsgHandle::set_odom_info_callback(OdomInfoCallback callback) {
  std::string topic_name;
  try {
    topic_name = input_topic_map_.at("odom_info");
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("odom_info tpoic not found in topic config file" + std::string(e.what()));
  }
  odom_info_sub_ = std::make_shared<ros::Subscriber>(nh_->subscribe<OdomInfoMsg>(topic_name, 10, callback));
}

void MsgHandle::set_obstacle_callback(ObstacleCallback callback) {
  std::string topic_name;
  try {
    topic_name = input_topic_map_.at("obstacle");
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("obstacle tpoic not found in topic config file" + std::string(e.what()));
  }
  obstacle_sub_ = std::make_shared<ros::Subscriber>(nh_->subscribe<ObstacleMsg>(topic_name, 10, callback));
}

void MsgHandle::set_task_info_callback(TaskInfoCallback callback) {
  std::string topic_name;
  try {
    topic_name = input_topic_map_.at("task_info");
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("task_info tpoic not found in topic config file" + std::string(e.what()));
  }
  task_info_sub_ = std::make_shared<ros::Subscriber>(nh_->subscribe<TaskInfoMsg>(topic_name, 10, callback));
}

void MsgHandle::set_joystick_callback(JoystickCallback callback) {
  std::string topic_name;
  try {
    topic_name = input_topic_map_.at("joystick");
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("joystick tpoic not found in topic config file" + std::string(e.what()));
  }
  joystick_sub_ = std::make_shared<ros::Subscriber>(nh_->subscribe<JoystickMsg>(topic_name, 10, callback));
}

void MsgHandle::publish_control_command(const ControlCommandMsg& control_command) {
  control_command_pub_->publish(control_command);
}
void MsgHandle::publish_plan_debug_info(const PlanDebugInfoMsg& plan_debug_info) {
  plan_debug_info_pub_->publish(plan_debug_info);
}
void MsgHandle::publish_navigation_status(const NavigationStatusMsg& navigation_status) {
  navigation_status_pub_->publish(navigation_status);
}

void MsgHandle::load_topic(const std::string& topic_config_file) {
  try {
    YAML::Node yaml_node = YAML::LoadFile(topic_config_file);
    std::cout << "load topic from topic_config_file: " << topic_config_file << std::endl;
    for (YAML::const_iterator topic_i = yaml_node["input_topics"].begin(); topic_i != yaml_node["input_topics"].end(); ++topic_i) {
      std::string topic_label = topic_i->first.as<std::string>();
      std::string topic_name = topic_i->second.as<std::string>();
      std::cout << "input topic_label: " << topic_label << ", topic_name: " << topic_name << std::endl;
      input_topic_map_.insert(std::pair<std::string, std::string>(topic_label, topic_name));
    }
    for (YAML::const_iterator topic_i = yaml_node["output_topics"].begin(); topic_i != yaml_node["output_topics"].end(); ++topic_i) {
      std::string topic_label = topic_i->first.as<std::string>();
      std::string topic_name = topic_i->second.as<std::string>();
      std::cout << "output topic_label: " << topic_label << ", topic_name: " << topic_name << std::endl;
      output_topic_map_.insert(std::pair<std::string, std::string>(topic_label, topic_name));
    }
  } catch (...) {
    throw std::runtime_error("Load topic file exception, file: " + topic_config_file);
  }
}

void MsgHandle::init_publisher() {
  try {
    control_command_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<ControlCommandMsg>(output_topic_map_.at("control_command"), 1));
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("control_command tpoic not found in topic config file" + std::string(e.what()));
  }
  try {
    plan_debug_info_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<PlanDebugInfoMsg>(output_topic_map_.at("plan_debug"), 1));
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("plan_debug tpoic not found in topic config file" + std::string(e.what()));
  }
  try {
    navigation_status_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<NavigationStatusMsg>(output_topic_map_.at("navigation_status"), 1));
  } catch (const std::out_of_range& e) {
    throw std::runtime_error("navigation_status tpoic not found in topic config file" + std::string(e.what()));
  }
}