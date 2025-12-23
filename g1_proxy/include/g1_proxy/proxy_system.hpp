// Heecheol Kim | heecheolkim@microsoft.com | MSRA Tokyo | 2025-12-23

#pragma once

#include <mutex>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace g1_proxy
{

class ProxySystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void ft_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  std::string arm_id_;
  
  std::vector<std::string> joint_names_;
  std::vector<double> pos_, vel_, eff_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;

  std::vector<std::string> sensor_names_;
  std::vector<double> sensor_ft_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;

  sensor_msgs::msg::JointState latest_state_;
  bool have_state_{false};
  std::mutex mtx_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_;
  geometry_msgs::msg::WrenchStamped latest_ft_;
  std::mutex mtx_ft_;
  bool have_ft_ = false;
};

}  // namespace g1_proxy
