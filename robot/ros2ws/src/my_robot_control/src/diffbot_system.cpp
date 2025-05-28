#include "my_robot_control/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_control
{
hardware_interface::CallbackReturn DiffBotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
//   for (const hardware_interface::ComponentInfo & joint : info_.joints)
//   {
//     // DiffBotSystem has exactly two states and one command interface on each joint
//     if (joint.command_interfaces.size() != 1)
//     {
//       RCLCPP_FATAL(
//         get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
//         joint.name.c_str(), joint.command_interfaces.size());
//       return hardware_interface::CallbackReturn::ERROR;
//     }

//     if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
//     {
//       RCLCPP_FATAL(
//         get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
//         joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
//         hardware_interface::HW_IF_VELOCITY);
//       return hardware_interface::CallbackReturn::ERROR;
//     }

//     if (joint.state_interfaces.size() != 2)
//     {
//       RCLCPP_FATAL(
//         get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
//         joint.state_interfaces.size());
//       return hardware_interface::CallbackReturn::ERROR;
//     }

//     if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
//     {
//       RCLCPP_FATAL(
//         get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
//         joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
//         hardware_interface::HW_IF_POSITION);
//       return hardware_interface::CallbackReturn::ERROR;
//     }

//     if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
//     {
//       RCLCPP_FATAL(
//         get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
//         joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
//         hardware_interface::HW_IF_VELOCITY);
//       return hardware_interface::CallbackReturn::ERROR;
//     }
//   }
  gpioInitialise();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
//   for (const auto & [name, descr] : joint_command_interfaces_)
//   {
//     set_command(name, get_state(name));
//   }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // TODO: Read actual hardware state and update state interfaces
  // For now, just return OK
  return hardware_interface::return_type::OK;
}

// In write()
hardware_interface::return_type DiffBotSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
    // Convert from rad/s to m/s (need your wheel radius)
    double wheel_radius = 0.03; // meters

    double left_vel_mps = get_command("left_wheel_joint/velocity") * wheel_radius;
    motor_driver_->setVelocity(left_motor_pwm_pin_, left_motor_forward_pin_, left_motor_backward_pin_, left_vel_mps);

    double right_vel_mps = get_command("right_wheel_joint/velocity") * wheel_radius;
    motor_driver_->setVelocity(right_motor_pwm_pin_, right_motor_forward_pin_, right_motor_backward_pin_, right_vel_mps);

    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
    PLUGINLIB_EXPORT_CLASS(
    my_robot_control::DiffBotSystem, hardware_interface::SystemInterface)
