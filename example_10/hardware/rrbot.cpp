// Copyright 2023 ros2_control Development Team
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

#include "ros2_control_demo_example_10/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_10
{
hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // check if we have same command and state interfaces for joint this makes iterating easier
  // first check if size is equal then we only need to iterate over one of them
  if (joint_state_interfaces_.size() != joint_command_interfaces_.size())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "Expect joint CommandInterface and joint StateInterfaces to be of equal size.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & [state_itf_name, state_itf_descr] : joint_state_interfaces_)
  {
    if (joint_command_interfaces_.find(state_itf_name) == joint_command_interfaces_.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Expect joint CommandInterface and joint StateInterfaces to be equal but StateInterface "
        "includes<%s> which is not included in CommandInterfaces.",
        state_itf_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // RRBotSystemWithGPIOHardware has exactly two GPIO components
  if (info_.gpios.size() != 2)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
      "RRBotSystemWithGPIOHardware has '%ld' GPIO components, '%d' expected.", info_.gpios.size(),
      2);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // with exactly 1 command interface
  for (int i = 0; i < 2; i++)
  {
    if (info_.gpios[i].command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "GPIO component %s has '%ld' command interfaces, '%d' expected.",
        info_.gpios[i].name.c_str(), info_.gpios[i].command_interfaces.size(), 1);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // and 3/1 state interfaces, respectively
  if (info_.gpios[0].state_interfaces.size() != 3)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
      "GPIO component %s has '%ld' state interfaces, '%d' expected.", info_.gpios[0].name.c_str(),
      info_.gpios[0].state_interfaces.size(), 3);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.gpios[1].state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
      "GPIO component %s has '%ld' state interfaces, '%d' expected.", info_.gpios[0].name.c_str(),
      info_.gpios[0].state_interfaces.size(), 1);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // set gpios
  flange_ios_ = std::make_unique<FlangeIOs>(info_.gpios[0].name);
  flange_vacuum_ = std::make_unique<FlangeVacuum>(info_.gpios[1].name);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Configuring ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [itf_name, itf_descr] : joint_state_interfaces_)
  {
    set_state(itf_name, 0.0);
    set_command(itf_name, 0.0);
  }
  for (const auto & [gpio_state, gpio_state_desc] : gpio_state_interfaces_)
  {
    set_state(gpio_state, 0.0);
  }
  for (const auto & [gpio_cmd, gpio_cmd_desc] : gpio_command_interfaces_)
  {
    set_command(gpio_cmd, 0.0);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [itf_name, itf_descr] : joint_state_interfaces_)
  {
    set_command(itf_name, get_state(itf_name));
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemWithGPIOHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Reading...");

  for (const auto & [itf_name, itf_descr] : joint_state_interfaces_)
  {
    // Simulate RRBot's movement
    set_state(itf_name, get_state(itf_name) + (get_command(itf_name) - get_state(itf_name)));
  }

  // mirror GPIOs back
  set_state(flange_ios_->out, get_command(flange_ios_->out));
  set_state(flange_vacuum_->vacuum, get_command(flange_vacuum_->vacuum));
  // random inputs
  unsigned int seed = time(NULL) + 1;
  set_state(flange_ios_->input_1, static_cast<float>(rand_r(&seed)));
  seed = time(NULL) + 2;
  set_state(flange_ios_->input_2, static_cast<float>(rand_r(&seed)));

  for (const auto & [gpio_state, gpio_state_desc] : gpio_state_interfaces_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Read %.1f from GP input %s!",
      get_state(gpio_state), gpio_state.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "GPIOs successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemWithGPIOHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Writing...");

  for (const auto & [gpio_cmd, gpio_cmd_desc] : gpio_command_interfaces_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Got command %.1f for GP output %s!",
      get_command(gpio_cmd), gpio_cmd.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "GPIOs successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_10

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_10::RRBotSystemWithGPIOHardware, hardware_interface::SystemInterface)
