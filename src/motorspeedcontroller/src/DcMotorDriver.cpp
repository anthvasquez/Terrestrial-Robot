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

#include <functional>
#include <memory>
#include <lgpio.h>
#include <unistd.h>
#include "utility.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
using namespace std;
extern const std::string SPEED_TOPIC;

class DcMotorDriver : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  int PwmPin = 12;
  const int PWM_FREQ = 10000;
  int pwmFd;

  void SetMotorSpeed(const std_msgs::msg::Int32& msg) const
  {
    auto queue = lgTxPwm(pwmFd, PwmPin, PWM_FREQ, msg.data, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Setting the speed to: %d%% (%d commands in queue)", msg.data, queue);
  }

  void OnShutdownCallback()
  {
    lgGpioFree(pwmFd, PwmPin);
    lgGpiochipClose(pwmFd);
  }

public:
  DcMotorDriver() : Node("dcmotordriver")
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The GPIO pin occupied by this motor.";
    param_desc.read_only = true;
    this->declare_parameter("PWM_PIN", 12, param_desc);

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        SPEED_TOPIC, 10, std::bind(&DcMotorDriver::SetMotorSpeed, this, _1));
    
    
    //Initialize pwm
    PwmPin = this->get_parameter("PWM_PIN").as_int();
    int pwmFd = lgGpiochipOpen(0);
    if(pwmFd < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open gpio chip 0.  Exiting...");
      rclcpp::shutdown();
    }
    if(lgGpioClaimOutput(pwmFd, 0, PwmPin, 0))
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to claim gpio pin %d.  Exiting...", PwmPin);
      rclcpp::shutdown();
    }
    rclcpp::on_shutdown(std::bind(&DcMotorDriver::OnShutdownCallback, this));
    RCLCPP_INFO(this->get_logger(), "%s started successfully on pin %d", this->get_name(), PwmPin);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DcMotorDriver>());
  rclcpp::shutdown();
  return 0;
}
