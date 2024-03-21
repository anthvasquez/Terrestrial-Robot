
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

namespace terrestrial_robot
{
class BDC_LM298_SystemHardware : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BDC_LM298_SystemHardware)

  TERRESTRIAL_ROBOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TERRESTRIAL_ROBOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  TERRESTRIAL_ROBOT_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  TERRESTRIAL_ROBOT_LOCAL
  hardware_interface::CallbackReturn InitializeMotor();

  TERRESTRIAL_ROBOT_LOCAL
  void SetPinSpeed(int hw_pin, int duty_cycle);

private:
  int pwm_freq;
  std::string name;
  int forward_pin;
  int backward_pin;
  double vel_cmd;
  double vel_state;
  int rpm;
  int pwmFd;
};
}  // namespace terrestrial_robot