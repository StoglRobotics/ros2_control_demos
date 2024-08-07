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

#include "ros2_control_demo_example_7/r6bot_hardware.hpp"
#include <string>
#include <vector>

namespace ros2_control_demo_example_7
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 6 joints and 2 interfaces
  for (const auto & [state_itf_name, state_itf_descr] : joint_state_interfaces_)
  {
    set_state(state_itf_name, 0.0);
  }
  for (const auto & [cmd_itf_name, cmd_itf_descr] : joint_command_interfaces_)
  {
    set_state(cmd_itf_name, 0.0);
  }

  for (const auto & [sensor_itf_name, sensor_itf_descr] : sensor_state_interfaces_)
  {
    set_state(sensor_itf_name, 0.0);
  }

  fts_sensor_ = std::make_unique<FTS_Sensor>(info_.sensors[0].name);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription>
RobotSystem::export_command_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> additional_cmd_itfs;
  additional_cmd_itfs.emplace_back(
    fts_sensor_->name, hardware_interface::InterfaceInfo("force.x", "double"));
  additional_cmd_itfs.emplace_back(
    fts_sensor_->name, hardware_interface::InterfaceInfo("force.y", "double"));
  additional_cmd_itfs.emplace_back(
    fts_sensor_->name, hardware_interface::InterfaceInfo("force.z", "double"));
  additional_cmd_itfs.emplace_back(
    fts_sensor_->name, hardware_interface::InterfaceInfo("torque.x", "double"));
  additional_cmd_itfs.emplace_back(
    fts_sensor_->name, hardware_interface::InterfaceInfo("torque.y", "double"));
  additional_cmd_itfs.emplace_back(
    fts_sensor_->name, hardware_interface::InterfaceInfo("torque.z", "double"));
  return additional_cmd_itfs;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber
  for (const auto & component : info_.joints)
  {
    const auto & joint = component.name;
    const auto joint_vel = joint + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
    const auto joint_pos = joint + "/" + std::string(hardware_interface::HW_IF_POSITION);

    set_state(joint_vel, get_command(joint_vel));
    set_state(joint_pos, get_state(joint_pos) + get_command(joint_vel) * period.seconds());
    set_state(joint_pos, get_command(joint_pos));
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_7::RobotSystem, hardware_interface::SystemInterface)
