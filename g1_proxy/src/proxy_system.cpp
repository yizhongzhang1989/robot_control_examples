// Heecheol Kim | heecheolkim@microsoft.com | MSRA Tokyo | 2025-12-23
// The G1 robot does not have a ros2_control hardware interface.
// This file implements a proxy hardware interface that bridges ros2_control
// with the actual robot via DDS topics.
// Architecture: hardware_interface <-> ros2_topic <-> g1_node.py <-> actual_robot
// Separate proxy hw interfaces are launched for left and right arms.

#include "g1_proxy/proxy_system.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace g1_proxy
{

hardware_interface::CallbackReturn ProxySystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Get arm_id from HardwareInfo parameters
  if (info_.hardware_parameters.find("arm_id") != info_.hardware_parameters.end()) {
    arm_id_ = info_.hardware_parameters.at("arm_id");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ProxySystem"), "arm_id parameter not found");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  joint_names_.clear();
  for (const auto & j : info_.joints) joint_names_.push_back(j.name);

  const size_t n = joint_names_.size();
  pos_.assign(n, 0.0);
  vel_.assign(n, 0.0);
  eff_.assign(n, 0.0);

  cmd_pos_.assign(n, 0.0);
  cmd_vel_.assign(n, 0.0);
  cmd_eff_.assign(n, 0.0);

  sensor_names_.clear();
  for (const auto & s : info_.sensors) sensor_names_.push_back(s.name);
  sensor_ft_.assign(sensor_names_.size() * 6, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ProxySystem::export_state_interfaces()
{
  // Provides pose and wrench (F/T) information that cartesian_controller requires
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(joint_names_.size() * 3);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &eff_[i]);
  }

  for (size_t i = 0; i < sensor_names_.size(); ++i) {
    si.emplace_back(sensor_names_[i], "force.x",  &sensor_ft_[i * 6 + 0]);
    si.emplace_back(sensor_names_[i], "force.y",  &sensor_ft_[i * 6 + 1]);
    si.emplace_back(sensor_names_[i], "force.z",  &sensor_ft_[i * 6 + 2]);
    si.emplace_back(sensor_names_[i], "torque.x", &sensor_ft_[i * 6 + 3]);
    si.emplace_back(sensor_names_[i], "torque.y", &sensor_ft_[i * 6 + 4]);
    si.emplace_back(sensor_names_[i], "torque.z", &sensor_ft_[i * 6 + 5]);
  }

  return si;
}

std::vector<hardware_interface::CommandInterface> ProxySystem::export_command_interfaces()
{
  // Receives commands published by cartesian_controller
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(joint_names_.size() * 3);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &cmd_eff_[i]);
  }
  return ci;
}

void ProxySystem::state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{  
  // Subscriber callback: receives latest joint state from g1_node
  std::lock_guard<std::mutex> lk(mtx_);
  latest_state_ = *msg;
  have_state_ = true;
}

void ProxySystem::ft_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{  
  // Subscriber callback: receives latest F/T sensor state from g1_node
  std::lock_guard<std::mutex> lk(mtx_ft_);
  latest_ft_ = *msg;
  have_ft_ = true;
}

hardware_interface::CallbackReturn ProxySystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  node_ = std::make_shared<rclcpp::Node>("proxy_hw_node_" + arm_id_);

  rclcpp::QoS qos(10);
  qos.reliable();

  std::string state_topic = "/robot/" + arm_id_ + "/state";
  std::string ft_topic = "/robot/" + arm_id_ + "/ft";
  std::string cmd_topic = "/robot/" + arm_id_ + "/command";

  sub_state_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic, qos,
    std::bind(&ProxySystem::state_cb, this, std::placeholders::_1));

  sub_ft_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    ft_topic, qos,
    std::bind(&ProxySystem::ft_cb, this, std::placeholders::_1));

  pub_cmd_ = node_->create_publisher<sensor_msgs::msg::JointState>(cmd_topic, qos);
  
  RCLCPP_INFO(rclcpp::get_logger("ProxySystem"), 
    "Proxy activated for arm: %s", arm_id_.c_str());
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProxySystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  sub_state_.reset();
  sub_ft_.reset();
  pub_cmd_.reset();
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProxySystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Copy latest joint state and F/T sensor data from callbacks into the interface
  // so cartesian_controller can read them
  if (node_) {
    rclcpp::spin_some(node_);
  }
  
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (have_state_) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto & name = joint_names_[i];
        for (size_t k = 0; k < latest_state_.name.size(); ++k) {
          if (latest_state_.name[k] == name) {
            if (k < latest_state_.position.size()) pos_[i] = latest_state_.position[k];
            if (k < latest_state_.velocity.size()) vel_[i] = latest_state_.velocity[k];
            if (k < latest_state_.effort.size())   eff_[i] = latest_state_.effort[k];
            break;
          }
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lk_ft(mtx_ft_);
    if (have_ft_ && !sensor_names_.empty()) {
      sensor_ft_[0] = latest_ft_.wrench.force.x;
      sensor_ft_[1] = latest_ft_.wrench.force.y;
      sensor_ft_[2] = latest_ft_.wrench.force.z;
      sensor_ft_[3] = latest_ft_.wrench.torque.x;
      sensor_ft_[4] = latest_ft_.wrench.torque.y;
      sensor_ft_[5] = latest_ft_.wrench.torque.z;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProxySystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Publish commands from cartesian_controller to g1_node
  if (!pub_cmd_) return hardware_interface::return_type::OK;

  sensor_msgs::msg::JointState msg;
  msg.name = joint_names_;
  msg.position = cmd_pos_;
  msg.velocity = cmd_vel_;
  msg.effort = cmd_eff_;
  pub_cmd_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace g1_proxy

PLUGINLIB_EXPORT_CLASS(g1_proxy::ProxySystem, hardware_interface::SystemInterface)
