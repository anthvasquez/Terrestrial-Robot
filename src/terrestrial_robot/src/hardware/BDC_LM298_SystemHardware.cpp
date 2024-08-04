// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "hardware/BDC_LM298_SystemHardware.hpp"

#include <boost/algorithm/string.hpp>
#include <lgpio.h>
#include <math.h>
#include <algorithm>
#include <cmath>

namespace terrestrial_robot
{
const int wheel_diameter_mm = 97;
const int supply_voltage = 12;
static const rclcpp::Logger logger = logger;

hardware_interface::CallbackReturn BDC_LM298_SystemHardware::InitializeMotor()
{
  if (lgGpioClaimOutput(pwmFd, 0, forward_pin, 0))
  {
    RCLCPP_ERROR(logger, "Unable to claim gpio pin %d.  Exiting...", forward_pin);
    return hardware_interface::CallbackReturn::FAILURE;
  }

  if (lgGpioClaimOutput(pwmFd, 0, backward_pin, 0))
  {
    RCLCPP_ERROR(logger, "Unable to claim gpio pin %d.  Exiting...", backward_pin);
    return hardware_interface::CallbackReturn::FAILURE;
  }

  SetPinSpeed(forward_pin, 0);
  SetPinSpeed(backward_pin, 0);

  RCLCPP_INFO(logger, "%s motor started successfully on pin %d & %d", name.c_str(), forward_pin, backward_pin);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void BDC_LM298_SystemHardware::SetPinSpeed(int hw_pin, int duty_cycle)
{
  auto queue = lgTxPwm(pwmFd, hw_pin, pwm_freq, duty_cycle, 0, 0);
  if(queue >= 0)
    RCLCPP_DEBUG(logger, "Setting pin %d speed to: %d%% (%d commands in queue)", hw_pin, duty_cycle, queue);
  else
    RCLCPP_DEBUG(logger, "Error setting pin %d to %d%%", hw_pin, duty_cycle);
}

#pragma region Export Interfaces

std::vector<hardware_interface::StateInterface> BDC_LM298_SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.push_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_state));
  return state_interfaces;
}

/**
 * This method assumes on_init() has been run and all the joints and commands are as expected
 */
std::vector<hardware_interface::CommandInterface> BDC_LM298_SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.push_back(
      hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_cmd));

  return command_interfaces;
}

#pragma endregion

hardware_interface::CallbackReturn BDC_LM298_SystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1)
  {
    RCLCPP_FATAL(logger, "Wrong number of joints specified.  Expected: 1, Actual: %d", (int)info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto joint = info_.joints[0];
  auto commands = joint.command_interfaces;

  if (commands.size() != 1 && commands[0].name != hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_FATAL(logger, "Joint %s must specify exactly one velocity command interface", joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.name.empty())
  {
    RCLCPP_FATAL(logger, "Joint for tag %s does not specify a name", info_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters["pwm_freq"].empty())
  {
    RCLCPP_FATAL(logger,
                 "Must specify the 'pwm_freq' as a hardware "
                 "parameter.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters["forward_pin"].empty())
  {
    RCLCPP_FATAL(logger, "Joint '%s' must specify a forward pin number.", joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters["backward_pin"].empty())
  {
    RCLCPP_FATAL(logger, "Joint '%s' must specify a backward pin number.", joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters["rpm"].empty())
  {
    RCLCPP_FATAL(logger, "Joint '%s' must specify a motor rpm.", joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

#pragma region Inactive Transitions

hardware_interface::CallbackReturn BDC_LM298_SystemHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    auto joint = info_.joints[0];
    name = joint.name;
    pwm_freq = stoi(info_.hardware_parameters["pwm_freq"]);
    forward_pin = stoi(info_.hardware_parameters["forward_pin"]);
    backward_pin = stoi(info_.hardware_parameters["backward_pin"]);
    rpm = stoi(info_.hardware_parameters["rpm"]);
    vel_cmd = 0.0;
    pwmFd = lgGpiochipOpen(0);
    if (pwmFd < 0)
    {
      RCLCPP_ERROR(logger, "Unable to open gpio chip 0.  Exiting...");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_INFO(logger, "pwm_freq: %s, forward_pin: %s, backward_pin: %s",
                info_.hardware_parameters["pwm_freq"].c_str(), info_.hardware_parameters["forward_pin"].c_str(),
                info_.hardware_parameters["backward_pin"].c_str());
    RCLCPP_FATAL(logger, "Could not parse int from %s hardware parameters.", info_.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BDC_LM298_SystemHardware::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  pwm_freq = 10000;  // default
  lgGpiochipClose(pwmFd);
  return hardware_interface::CallbackReturn::SUCCESS;
}

#pragma endregion

#pragma region Active Transitions

hardware_interface::CallbackReturn BDC_LM298_SystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return InitializeMotor();
}

hardware_interface::CallbackReturn
BDC_LM298_SystemHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  SetPinSpeed(forward_pin, 0);
  SetPinSpeed(backward_pin, 0);
  lgGpioFree(pwmFd, forward_pin);
  lgGpioFree(pwmFd, backward_pin);

  return hardware_interface::CallbackReturn::SUCCESS;
}

#pragma endregion

hardware_interface::CallbackReturn BDC_LM298_SystemHardware::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(logger, "Shutting down actuator %s", info_.name.c_str());
  if (previous_state.label() == "Active")
  {
    // set all motors to speed zero
    on_deactivate(previous_state);
  }
  on_cleanup(previous_state);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Read values from actuator and set state interfaces
hardware_interface::return_type BDC_LM298_SystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  vel_state = vel_cmd;
  return hardware_interface::return_type::OK;
}

// Read command interfaces and write to actuator
hardware_interface::return_type BDC_LM298_SystemHardware::write(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
{
  int motorVoltage;
  motorVoltage = rpm >= 1000 ? 24 : 12;
  double circumference_mm = M_PI * wheel_diameter_mm;
  double max_wheel_speed_mps = (rpm * circumference_mm) / (1000 * 60);
  int duty_cycle = 100 * vel_cmd / max_wheel_speed_mps;
  duty_cycle *= motorVoltage / supply_voltage;
  duty_cycle = std::clamp(duty_cycle, -100, 100);
  std::string logger = "BDC_LM298_SystemHardware " + name;
  
  if (duty_cycle >= 0)
  {
    RCLCPP_INFO(rclcpp::get_logger(logger), "Writing FW: %d BW: %d to %s with %d rpm", duty_cycle, 0, name.c_str(), rpm);
    SetPinSpeed(forward_pin, duty_cycle);
    SetPinSpeed(backward_pin, 0);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger(logger), "Writing FW: %d BW: %d to %s with %d rpm", 0, std::abs(duty_cycle), name.c_str(), rpm);
    SetPinSpeed(forward_pin, 0);
    SetPinSpeed(backward_pin, std::abs(duty_cycle));
  }

  return hardware_interface::return_type::OK;
}

}  // namespace terrestrial_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(terrestrial_robot::BDC_LM298_SystemHardware, hardware_interface::ActuatorInterface)
