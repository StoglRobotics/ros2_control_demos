// Copyright 2020 ros2_control Development Team
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

#include "ros2_control_demo_example_8/rrbot_transmissions_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

namespace ros2_control_demo_example_8
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("RRBotTransmissionsSystemPositionOnlyHardware"));

  clock_ = std::make_unique<rclcpp::Clock>();

  RCLCPP_INFO(*logger_, "Initializing...");

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  actuator_slowdown_ = hardware_interface::stod(info_.hardware_parameters["actuator_slowdown"]);

  // create transmissions, joint and actuator handles
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

  for (const auto & transmission_info : info_.transmissions)
  {
    // only simple transmissions are supported in this demo
    if (transmission_info.type != "transmission_interface/SimpleTransmission")
    {
      RCLCPP_FATAL(
        *logger_, "Transmission '%s' of type '%s' not supported in this demo",
        transmission_info.name.c_str(), transmission_info.type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::shared_ptr<transmission_interface::Transmission> transmission;
    try
    {
      transmission = transmission_loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
      RCLCPP_FATAL(
        *logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (transmission_info.actuators.size() != transmission_info.joints.size())
    {
      RCLCPP_FATAL(
        *logger_,
        "For transmission %s: the size of the actuators is %li but the size of the joints is %li. "
        "Expected to be the same.",
        transmission_info.name.c_str(), transmission_info.actuators.size(),
        transmission_info.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::vector<std::shared_ptr<transmission_interface::JointHandle>> joint_handles_vec;
    joint_handles_.reserve(transmission_info.actuators.size());
    std::vector<std::shared_ptr<transmission_interface::ActuatorHandle>> actuator_handles_vec;
    actuator_handles_vec.reserve(transmission_info.actuators.size());

    for (size_t i = 0; i < transmission_info.actuators.size(); ++i)
    {
      // this demo supports only one interface per joint
      if (!(transmission_info.joints.at(i).state_interfaces.size() == 1 &&
            transmission_info.joints.at(i).state_interfaces[0] ==
              hardware_interface::HW_IF_POSITION &&
            transmission_info.joints.at(i).command_interfaces.size() == 1 &&
            transmission_info.joints.at(i).command_interfaces[0] ==
              hardware_interface::HW_IF_POSITION))
      {
        RCLCPP_FATAL(
          *logger_, "Invalid transmission joint '%s' configuration for this demo",
          transmission_info.joints.at(i).name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      const auto & joint_name = transmission_info.joints.at(i).name;
      const auto & interface_type = std::string(hardware_interface::HW_IF_POSITION);
      std::shared_ptr<transmission_interface::JointHandle> joint_handle =
        std::make_shared<transmission_interface::JointHandle>(
          hardware_interface::InterfaceDescription(
            joint_name, hardware_interface::InterfaceInfo(interface_type, "double")));

      // check that transmission interface name is present in the StateInterfaces and
      // CommandInterfaces.
      if (joint_state_interfaces_.find(joint_handle->get_name()) == joint_state_interfaces_.end())
      {
        if (joint_state_interfaces_.find(joint_handle->get_name()) == joint_state_interfaces_.end())
          RCLCPP_FATAL(
            *logger_,
            "A interface '%s' is expected to be present in joint_state_interfaces_ as its used "
            "for JointHandle but was not found.",
            joint_handle->get_name().c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else if (
        joint_command_interfaces_.find(joint_handle->get_name()) == joint_command_interfaces_.end())
      {
        RCLCPP_FATAL(
          *logger_,
          "A interface '%s' is expected to be present in joint_command_interfaces_ as its used "
          "for JointHandle but was not found.",
          joint_handle->get_name().c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      joint_handles_vec.push_back(joint_handle);
      const auto & [jh_it, jh_succ] =
        joint_handles_.insert({joint_handle->get_name(), joint_handle});
      if (!jh_succ)
      {
        RCLCPP_FATAL(
          *logger_,
          "Could not insert the JointHandle with name '%s' into map. Check for duplicate "
          "names.",
          joint_handle->get_name().c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // no check for actuators types
      const auto & actuator_name = transmission_info.actuators.at(i).name;
      std::shared_ptr<transmission_interface::ActuatorHandle> actuator_handle =
        std::make_shared<transmission_interface::ActuatorHandle>(
          hardware_interface::InterfaceDescription(
            actuator_name, hardware_interface::InterfaceInfo(interface_type, "double")));

      actuator_handles_vec.push_back(actuator_handle);
      const auto & [ah_it, ah_succ] =
        actuator_handles_.insert({actuator_handle->get_name(), actuator_handle});
      if (!ah_succ)
      {
        RCLCPP_FATAL(
          *logger_,
          "Could not insert the ActuatorHandle with name '%s' into map. Check for duplicate "
          "names.",
          joint_handle->get_name().c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      /// @note no need to store the joint and actuator handles, the transmission
      /// will keep whatever info it needs after is done with them
      try
      {
        transmission->configure(joint_handles_vec, actuator_handles_vec);
      }
      catch (const transmission_interface::TransmissionInterfaceException & exc)
      {
        RCLCPP_FATAL(
          *logger_, "Error while configuring %s: %s", transmission_info.name.c_str(), exc.what());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // This is only ok because we know that only one joint per transmission is there and joint
      // name and transmission joint name are the same. If there are multiple joints/actuators per
      // transmission this has to be changed
      auto actuator_state = std::make_shared<hardware_interface::StateInterface>(
        hardware_interface::InterfaceDescription(
          actuator_name, hardware_interface::InterfaceInfo(interface_type, "double")));
      actuator_states_[actuator_state->get_name()] = actuator_state;
      transmissions_[joint_name] = transmission;
      joint_to_transmission_info_[joint_name] = transmission_info;
      joint_to_actuator_[joint_name] = actuator_name;
      actuator_to_joint_[actuator_name] = joint_name;
    }
  }

  RCLCPP_INFO(*logger_, "Initialization successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Configuring...");

  for (const auto & joint : info_.joints)
  {
    const auto joint_pos = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    const auto actuator_name = joint_to_actuator_.at(joint.name);
    const auto act_pos = actuator_name + "/" + hardware_interface::HW_IF_POSITION;
    set_state(joint_pos, 0.0);
    set_command(joint_pos, 0.0);
    actuator_states_.at(act_pos)->set_value(0.0);
    joint_handles_.at(joint_pos)->set_value(kNaN);
    actuator_handles_.at(joint_pos)->set_value(kNaN);
  }

  RCLCPP_INFO(*logger_, "Configuration successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Activating...");
  RCLCPP_INFO(*logger_, "Activation successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Deactivating...");
  RCLCPP_INFO(*logger_, "Deactivation successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotTransmissionsSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & joint : info_.joints)
  {
    const auto & actuator_name = joint_to_actuator_.at(joint.name);
    const auto act_pos = actuator_name + "/" + hardware_interface::HW_IF_POSITION;
    const auto joint_pos = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    // actuator: state -> transmission
    actuator_handles_.at(act_pos)->set_value(actuator_states_.at(act_pos)->get_value<double>());

    // transmission: actuator -> joint
    transmissions_.at(joint.name)->actuator_to_joint();

    // joint: transmission -> state
    set_state(joint_pos, joint_handles_.at(joint_pos)->get_value<double>());
    // log state data
    // again, this only for simple transmissions, we know there is only one joint
    const auto & transmission_info = joint_to_transmission_info_.at(joint.name);
    std::stringstream ss;
    ss << "State data:" << std::endl
       << "\t" << joint_pos << ": " << get_state(joint_pos) << " <-- " << transmission_info.name
       << "(R=" << transmission_info.joints[0].mechanical_reduction << ") <-- " << actuator_name
       << ": " << actuator_states_.at(actuator_name)->get_value<double>();
    RCLCPP_INFO_THROTTLE(*logger_, *clock_, 1000, "%s", ss.str().c_str());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotTransmissionsSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & joint : info_.joints)
  {
    const auto & actuator_name = joint_to_actuator_.at(joint.name);
    const auto act_pos = actuator_name + "/" + hardware_interface::HW_IF_POSITION;
    const auto joint_pos = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    // joint: command -> transmission
    joint_handles_.at(joint_pos)->set_value(get_command(joint_pos));

    // transmission: joint -> actuator
    transmissions_.at(joint.name)->joint_to_actuator();

    // actuator: transmission -> command
    actuator_states_.at(act_pos)->set_value(actuator_handles_.at(act_pos)->get_value<double>());

    // simulate motor motion
    auto current_actuator_state = actuator_states_.at(act_pos)->get_value<double>();
    double new_state =
      current_actuator_state +
      (actuator_handles_.at(act_pos)->get_value<double>() - current_actuator_state) /
        actuator_slowdown_;
    actuator_states_.at(act_pos)->set_value(new_state);

    // log command data
    const auto & transmission_info = joint_to_transmission_info_.at(joint.name);
    std::stringstream ss;
    ss << "Command data:" << std::endl
       << "\t" << joint_pos << ": " << get_command(joint_pos) << " --> " << transmission_info.name
       << "(R=" << transmission_info.joints[0].mechanical_reduction << ") --> " << actuator_name
       << ": " << actuator_states_.at(actuator_name)->get_value<double>();

    RCLCPP_INFO_THROTTLE(*logger_, *clock_, 1000, "%s", ss.str().c_str());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_8

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_8::RRBotTransmissionsSystemPositionOnlyHardware,
  hardware_interface::SystemInterface)
