#ifndef MECANUM_CONTROLLER_HPP
#define MECANUM_CONTROLLER_HPP

#include <unordered_map>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"
#include "controller_parameters.hpp"

namespace mecanum_controller
{
using CmdType = geometry_msgs::msg::Twist;

class MecanumController : public controller_interface::ControllerInterface
{
public:
  MECANUM_CONTROLLER_PUBLIC
  MecanumController();

  MECANUM_CONTROLLER_PUBLIC
  ~MecanumController() = default;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  MECANUM_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();

  std::vector<std::string> joint_names_;
  const std::string INTERFACE_NAME = hardware_interface::HW_IF_VELOCITY;

  std::vector<std::string> command_interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  double wheel_separation_width;
  double wheel_separation_length;
  std::unordered_map<std::string, int> joint_map;
};
}  // namespace mecanum_controller

#endif  // MECANUM_CONTROLLER_HPP