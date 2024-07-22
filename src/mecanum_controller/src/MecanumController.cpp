#include <memory>
#include <string>
#include <vector>

#include "mecanum_controller/MecanumController.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace mecanum_controller
{
MecanumController::MecanumController()
  : controller_interface::ControllerInterface(), command_interface_types_(4), rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr)
{
}

void MecanumController::declare_parameters()
{
  // If validation fails, this will throw an exception
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn MecanumController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.front_left_joint.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'front_left_joint' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.front_right_joint.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'front_right_joint' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.back_left_joint.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'back_left_joint' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.back_right_joint.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'back_right_joint' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_map[params_.front_left_joint] = 0;
  joint_map[params_.front_right_joint] = 1;
  joint_map[params_.back_left_joint] = 2;
  joint_map[params_.back_right_joint] = 3;

  command_interface_types_[joint_map[params_.front_left_joint]] = params_.front_left_joint + "/" + INTERFACE_NAME;
  command_interface_types_[joint_map[params_.front_right_joint]] = params_.front_right_joint + "/" + INTERFACE_NAME;
  command_interface_types_[joint_map[params_.back_left_joint]] = params_.back_left_joint + "/" + INTERFACE_NAME;
  command_interface_types_[joint_map[params_.back_right_joint]] = params_.back_right_joint + "/" + INTERFACE_NAME;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  wheel_separation_width = params_.distance_left_to_right_wheel / 2.0;
  wheel_separation_length = params_.distance_front_to_rear_wheel / 2.0;

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MecanumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::CallbackReturn MecanumController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto twist_command = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!twist_command || !(*twist_command))
  {
    return controller_interface::return_type::OK;
  }

  double front_left_command, front_right_command, back_left_command, back_right_command;
  auto twist = *twist_command;

  front_left_command = (1 / params_.wheel_radius) * (twist->linear.x - twist->linear.y - (wheel_separation_width + wheel_separation_length) * twist->angular.z);
  front_right_command = (1 / params_.wheel_radius) * (twist->linear.x + twist->linear.y + (wheel_separation_width + wheel_separation_length) * twist->angular.z);
  back_left_command = (1 / params_.wheel_radius) * (twist->linear.x + twist->linear.y - (wheel_separation_width + wheel_separation_length) * twist->angular.z);
  back_right_command = (1 / params_.wheel_radius) * (twist->linear.x - twist->linear.y + (wheel_separation_width + wheel_separation_length) * twist->angular.z);

  //flip wheel direction since motors are reflected (backwards) on the right side
  front_right_command *= -1;
  back_right_command *= -1;

  command_interfaces_[joint_map[params_.front_left_joint]].set_value(front_left_command);
  command_interfaces_[joint_map[params_.front_right_joint]].set_value(front_right_command);
  command_interfaces_[joint_map[params_.back_left_joint]].set_value(back_left_command);
  command_interfaces_[joint_map[params_.back_right_joint]].set_value(back_right_command);

  return controller_interface::return_type::OK;
}

}  // namespace mecanum_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_controller::MecanumController, controller_interface::ControllerInterface)