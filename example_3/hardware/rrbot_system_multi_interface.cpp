// Copyright 2021 Department of Engineering Cybernetics, NTNU
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

#include "ros2_control_demo_example_3/rrbot_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_3
{
hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // check if we have same command and state interfaces for joint this makes iterating easier
  // first check if size is equal then we only need to iterate over one of them
  if (joint_state_interfaces_.size() != joint_command_interfaces_.size())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "Expect joint CommandInterface and joint StateInterfaces to be of equal size.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & [state_itf_name, state_itf_descr] : joint_state_interfaces_)
  {
    if (joint_command_interfaces_.find(state_itf_name) == joint_command_interfaces_.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Expect joint CommandInterface and joint StateInterfaces to be equal but StateInterface "
        "includes<%s> which is not included in CommandInterfaces.",
        state_itf_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // fill joints vector
  for (const auto & joint : info_.joints)
  {
    joints_.emplace_back(joint.name);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes.push_back(integration_level_t::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes.push_back(integration_level_t::VELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        new_modes.push_back(integration_level_t::ACCELERATION);
      }
    }
  }
  // Example criteria: All joints must be given new command mode at the same time
  if (new_modes.size() != info_.joints.size())
  {
    return hardware_interface::return_type::ERROR;
  }
  // Example criteria: All joints must have the same command mode
  if (!std::all_of(
        new_modes.begin() + 1, new_modes.end(),
        [&](integration_level_t mode) { return mode == new_modes[0]; }))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        set_command(key + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        set_command(key + "/" + hardware_interface::HW_IF_ACCELERATION, 0.0);
        control_level_ = integration_level_t::UNDEFINED;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_ != integration_level_t::UNDEFINED)
    {
      // Something else is using the joint! Abort!
      return hardware_interface::return_type::ERROR;
    }
    control_level_ = new_modes[0];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Activating... please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Set some default values
  // we checked that joint_state_interfaces_ == joint_command_interfaces_
  for (const auto & [itf_name, itf_descr] : joint_state_interfaces_)
  {
    if (!state_holds_value(itf_name) || std::isnan(get_state(itf_name)))
    {
      set_state(itf_name, 0.0);
    }
    if (!command_holds_value(itf_name) || std::isnan(get_command(itf_name)))
    {
      set_command(itf_name, 0.0);
    }
  }

  control_level_ = integration_level_t::UNDEFINED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "System successfully activated! %u",
    control_level_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Deactivating... please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // we checked that joint_state_interfaces_ == joint_command_interfaces_
  for (const auto & joint : joints_)
  {
    switch (control_level_)
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
          "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case integration_level_t::POSITION:
        set_state(joint.acceleration(), 0.0);
        set_state(joint.velocity(), 0.0);
        set_state(
          joint.position(),
          get_state(joint.position()) +
            (get_command(joint.position()) - get_state(joint.position()) / hw_slowdown_));
        break;
      case integration_level_t::VELOCITY:
        set_state(joint.acceleration(), 0.0);
        set_state(joint.velocity(), get_command(joint.velocity()));
        set_state(
          joint.position(), get_state(joint.position()) +
                              (get_state(joint.velocity()) * period.seconds()) / hw_slowdown_);
        break;
      case integration_level_t::ACCELERATION:
        set_state(joint.acceleration(), get_command(joint.acceleration()));
        set_state(
          joint.velocity(),
          get_state(joint.velocity()) +
            (get_command(joint.acceleration()) * period.seconds()) / hw_slowdown_);
        set_state(
          joint.position(), get_state(joint.position()) +
                              (get_state(joint.velocity()) * period.seconds()) / hw_slowdown_);
        break;
    }
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "Got pos: %.5f, vel: %.5f, acc: %.5f for joint %s!", get_state(joint.position()),
      get_state(joint.velocity()), get_state(joint.acceleration()), joint.name().c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (const auto & joint : joints_)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "Got the commands pos: %.5f, vel: %.5f, acc: %.5f for joint %s, control_lvl:%u",
      get_state(joint.position()), get_state(joint.velocity()), get_state(joint.acceleration()),
      joint.name().c_str(), control_level_);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_3

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_3::RRBotSystemMultiInterfaceHardware,
  hardware_interface::SystemInterface)
