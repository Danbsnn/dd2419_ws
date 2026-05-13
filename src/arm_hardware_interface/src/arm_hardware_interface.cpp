#include "arm_hardware_interface/arm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <mutex>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware_interface
{

static double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static double rad2deg(double r) { return r * 180.0 / M_PI; }
static double deg2rad(double d) { return d * M_PI / 180.0; }

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  latest_positions_.fill(0.0);

  command_time_ms_ = info_.hardware_parameters.count("command_time_ms")
                       ? std::stoi(info_.hardware_parameters.at("command_time_ms"))
                       : 100;

  startup_time_ms_ = info_.hardware_parameters.count("startup_time_ms")
                       ? std::stoi(info_.hardware_parameters.at("startup_time_ms"))
                       : 1500;

  feedback_received_ = false;
  first_write_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i)
    out.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_POSITION,
                     &hw_positions_[i]);
  return out;
}

// ---------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i)
    out.emplace_back(info_.joints[i].name,
                     hardware_interface::HW_IF_POSITION,
                     &hw_commands_[i]);
  return out;
}

// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  node_ = std::make_shared<rclcpp::Node>("arm_hw_interface");

  control_pub_ = node_->create_publisher<robp_interfaces::msg::ArmControl>(
      "/arm/control", 10);

  feedback_sub_ = node_->create_subscription<robp_interfaces::msg::ArmFeedback>(
      "/arm/feedback", 10,
      [this](robp_interfaces::msg::ArmFeedback::SharedPtr msg)
      {
        feedback_callback(msg);
      });

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// FIXED on_activate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });

  RCLCPP_INFO(node_->get_logger(), "Waiting for first feedback...");

  while (rclcpp::ok() && !feedback_received_)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!rclcpp::ok())
    return hardware_interface::CallbackReturn::ERROR;

  // FIX: copy array → vector
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (size_t i = 0; i < hw_positions_.size(); ++i)
      hw_positions_[i] = latest_positions_[i];
  }

  hw_commands_ = hw_positions_;
  first_write_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  rclcpp::shutdown();
  if (spin_thread_.joinable())
    spin_thread_.join();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// FIXED read()
// ---------------------------------------------------------------------------
hardware_interface::return_type
ArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);

  // FIX: copy array → vector
  for (size_t i = 0; i < hw_positions_.size(); ++i)
    hw_positions_[i] = latest_positions_[i];

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
hardware_interface::return_type
ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double j1 = rad2deg(joint_cmd("joint1"));
  double j2 = rad2deg(joint_cmd("joint2"));
  double j3 = rad2deg(joint_cmd("joint3"));
  double j4 = rad2deg(joint_cmd("joint4"));
  double j5 = rad2deg(joint_cmd("joint5"));
  double r  = rad2deg(joint_cmd("r_joint"));

  if (std::isnan(j1) || std::isnan(j2) || std::isnan(j3) ||
      std::isnan(j4) || std::isnan(j5) || std::isnan(r))
    return hardware_interface::return_type::OK;

  robp_interfaces::msg::ArmControl msg;

  msg.position = {
      (float)clamp(2 * (r + 84), 0, 240),
      (float)clamp(j5 + 120, 0, 240),
      (float)clamp(j4 + 120, 0, 240),
      (float)clamp(120 - j3, 0, 240),
      (float)clamp(j2 + 120, 0, 240),
      (float)clamp(j1 + 120, 0, 240)};

  int t = first_write_ ? startup_time_ms_ : command_time_ms_;

  // FIX: no narrowing
  msg.time = {
      static_cast<uint16_t>(t),
      static_cast<uint16_t>(t),
      static_cast<uint16_t>(t),
      static_cast<uint16_t>(t),
      static_cast<uint16_t>(t),
      static_cast<uint16_t>(t)};

  first_write_ = false;

  control_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
void ArmHardwareInterface::feedback_callback(
    const robp_interfaces::msg::ArmFeedback::SharedPtr msg)
{
  if (msg->position.size() < 6)
    return;

  std::array<double, 6> pos;

  pos[joint_index("joint1")] = deg2rad(msg->position[5] - 120);
  pos[joint_index("joint2")] = deg2rad(msg->position[4] - 120);
  pos[joint_index("joint3")] = deg2rad(-msg->position[3] + 120);
  pos[joint_index("joint4")] = deg2rad(msg->position[2] - 120);
  pos[joint_index("joint5")] = deg2rad(msg->position[1] - 120);
  pos[joint_index("r_joint")] = deg2rad(msg->position[0] / 2 - 84);

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    latest_positions_ = pos;
    feedback_received_ = true;
  }
}

// ---------------------------------------------------------------------------
std::size_t ArmHardwareInterface::joint_index(const std::string & name) const
{
  for (size_t i = 0; i < info_.joints.size(); ++i)
    if (info_.joints[i].name == name)
      return i;
  throw std::runtime_error("Unknown joint");
}

double ArmHardwareInterface::joint_cmd(const std::string & name) const
{
  return hw_commands_[joint_index(name)];
}

} // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::ArmHardwareInterface,
    hardware_interface::SystemInterface)