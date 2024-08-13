// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//

#include "ros2_control_demo_example_6/rrbot_actuator.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_6
{
hardware_interface::CallbackReturn RRBotModularJoint::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  const hardware_interface::ComponentInfo & joint = info_.joints[0];
  // RRBotModularJoint has exactly one state and command interface on each joint
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotModularJoint"),
      "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotModularJoint"),
      "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
      joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotModularJoint"), "Joint '%s' has %zu state interface. 1 expected.",
      joint.name.c_str(), joint.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotModularJoint"), "Joint '%s' have %s state interface. '%s' expected.",
      joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
      hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // assert that joint states and commands are the same so wie can later just iterate over one
  for (const auto & [state_itf_name, state_itf_descr] : joint_state_interfaces_)
  {
    if (joint_command_interfaces_.find(state_itf_name) == joint_command_interfaces_.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotModularJoint"),
        "Joint '%s' has state interface with name<%s> but not found in CommandInterfaces. Make "
        "sure the configuration exports same State- and CommandInterfaces",
        joint.name.c_str(), state_itf_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotModularJoint::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values for joints
  for (const auto & [cmd_itf_name, cmd_itf_descr] : joint_command_interfaces_)
  {
    if (!command_holds_value(cmd_itf_name) || std::isnan(get_command(cmd_itf_name)))
    {
      set_command(cmd_itf_name, 0.0);
    }
  }

  for (const auto & [state_itf_name, state_itf_descr] : joint_state_interfaces_)
  {
    if (!command_holds_value(state_itf_name) || std::isnan(get_state(state_itf_name)))
    {
      set_state(state_itf_name, 0.0);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotModularJoint::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotModularJoint::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Reading...");

  // Simulate RRBot's movement
  /*
   * @pre joint_state_interfaces_ names and are subset of joint_command_interfaces_
   */
  for (const auto & [itf_name, itf_descr] : joint_state_interfaces_)
  {
    auto old_state = get_state(itf_name);
    auto old_cmd = get_command(itf_name);
    set_state(itf_name, old_state + (old_cmd - old_state) / hw_slowdown_);
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotModularJoint"), "Got state %.5f for joint '%s'!",
      get_state(itf_name), itf_name.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_6::RRBotModularJoint::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Writing...please wait...");

  // Simulate sending commands to the hardware
  for (const auto & [itf_name, itf_descr] : joint_command_interfaces_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotModularJoint"), "Got command %.5f for joint '%s'!",
      get_command(itf_name), itf_name.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotModularJoint"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_6

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_6::RRBotModularJoint, hardware_interface::ActuatorInterface)
