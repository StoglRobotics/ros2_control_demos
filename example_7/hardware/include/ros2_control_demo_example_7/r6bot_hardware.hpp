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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_HARDWARE_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_HARDWARE_HPP_

#include "memory"
#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace ros2_control_demo_example_7
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  std::vector<hardware_interface::InterfaceDescription> export_command_interface_descriptions()
    override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  struct FTS_Sensor
  {
    explicit FTS_Sensor(const std::string & sensor_name) : name(sensor_name) {}
    // delete move constructor since would throw because of const std::string members
    // but we dont want to move this anyways so const for member is ok i guess
    FTS_Sensor(FTS_Sensor && other) = delete;

    const std::string name;
    const std::string force_x = "force.x";
    const std::string force_y = "force.y";
    const std::string force_z = "force.z";
    const std::string torque_x = "torque.x";
    const std::string torque_y = "torque.y";
    const std::string torque_z = "torque.z";
  };

  std::unique_ptr<FTS_Sensor> fts_sensor_;
};

}  // namespace ros2_control_demo_example_7

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_HARDWARE_HPP_
