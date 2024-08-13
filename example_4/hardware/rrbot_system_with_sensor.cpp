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

#include "ros2_control_demo_example_4/rrbot_system_with_sensor.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_4
{
hardware_interface::CallbackReturn RRBotSystemWithSensorHardware::on_init(
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
  hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemWithSensor has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithSensorHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  // we checked before that joint_state_itfs_ == joint_command_itfs_;
  for (const auto & [itf_name, itf_desc] : joint_state_interfaces_)
  {
    set_state(itf_name, 0.0);
    set_command(itf_name, 0.0);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithSensorHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [itf_name, itf_desc] : joint_state_interfaces_)
  {
    set_command(itf_name, get_state(itf_name));
  }

  // set default value for sensor
  for (const auto & [sensor_itf_name, sensor_itf_desc] : sensor_state_interfaces_)
  {
    if (!state_holds_value(sensor_itf_name) || std::isnan(get_state(sensor_itf_name)))
    {
      set_state(sensor_itf_name, 0.0);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithSensorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemWithSensorHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Reading...please wait...");

  for (const auto & [itf_name, itf_desc] : joint_state_interfaces_)
  {
    // Simulate RRBot's movement
    auto old_state = get_state(itf_name);
    auto new_state = old_state + (get_command(itf_name) - old_state) / hw_slowdown_;
    set_state(itf_name, new_state);
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Got state %.5f for joint %s!",
      get_state(itf_name), itf_name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Joints successfully read!");

  int i = 0;
  for (const auto & [sensor_itf_name, sensor_itf_desc] : sensor_state_interfaces_)
  {
    // Simulate RRBot's sensor data
    unsigned int seed = time(NULL) + i;
    set_state(
      sensor_itf_name,
      static_cast<float>(rand_r(&seed)) / (static_cast<float>(RAND_MAX / hw_sensor_change_)));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Got value %e for interface %s!",
      get_state(sensor_itf_name), sensor_itf_name.c_str());
    ++i;
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Sensors successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_4::RRBotSystemWithSensorHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Writing...please wait...");

  for (const auto & [itf_name, itf_desc] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Got command %.5f for joint %u!",
      get_command(itf_name), itf_name);
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithSensorHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_4

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_4::RRBotSystemWithSensorHardware, hardware_interface::SystemInterface)
