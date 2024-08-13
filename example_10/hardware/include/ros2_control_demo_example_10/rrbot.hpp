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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_10__RRBOT_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_10__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_demo_example_10/visibility_control.h"

namespace ros2_control_demo_example_10
{
class RRBotSystemWithGPIOHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemWithGPIOHardware);

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_DEMO_EXAMPLE_10_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the RRBot simulation

  struct FlangeVacuum
  {
    explicit FlangeVacuum(const std::string & gpio_name) : name(gpio_name) {}
    // delete move constructor since would throw because of const std::string members
    // but we dont want to move this anyways so const for member is ok i guess
    FlangeVacuum(FlangeVacuum && other) = delete;

    const std::string name;
    const std::string vacuum = name + "/vacuum";
  };

  struct FlangeIOs
  {
    explicit FlangeIOs(const std::string & gpio_name) : name(gpio_name) {}
    // delete move constructor since would throw because of const std::string members
    // but we dont want to move this anyways so const for member is ok i guess
    FlangeIOs(FlangeIOs && other) = delete;

    const std::string name;
    const std::string out = name + "/analog_output1";
    const std::string input_1 = name + "/analog_input1";
    const std::string input_2 = name + "/analog_input2";
  };
  std::unique_ptr<FlangeVacuum> flange_vacuum_;
  std::unique_ptr<FlangeIOs> flange_ios_;
};

}  // namespace ros2_control_demo_example_10

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_10__RRBOT_HPP_
