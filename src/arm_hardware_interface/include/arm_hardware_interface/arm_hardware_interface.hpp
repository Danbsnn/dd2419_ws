#pragma once

#include <array>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robp_interfaces/msg/arm_control.hpp"
#include "robp_interfaces/msg/arm_feedback.hpp"

namespace arm_hardware_interface
{

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // ── Lifecycle ─────────────────────────────────────────────────────────────
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ── Interfaces ────────────────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Control loop ──────────────────────────────────────────────────────────
  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS2 node running inside the plugin
  rclcpp::Node::SharedPtr node_;
  std::thread             spin_thread_;

  // Publishers / subscribers
  rclcpp::Publisher<robp_interfaces::msg::ArmControl>::SharedPtr   control_pub_;
  rclcpp::Subscription<robp_interfaces::msg::ArmFeedback>::SharedPtr feedback_sub_;

  // Shared state between the feedback callback and the read() call
  std::mutex           feedback_mutex_;
  std::array<double,6> latest_positions_{};
  bool                 feedback_received_{false};

  // ros2_control double buffers (one entry per joint, same order as info_.joints)
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;

  // Parameters read from the URDF <hardware> block
  int  command_time_ms_{100};
  int  startup_time_ms_{1500};
  bool first_write_{true};

  // Helpers
  void feedback_callback(const robp_interfaces::msg::ArmFeedback::SharedPtr msg);

  std::size_t joint_index(const std::string & name) const;
  double      joint_cmd  (const std::string & name) const;
};

}  // namespace arm_hardware_interface
