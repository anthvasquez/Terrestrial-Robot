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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
using namespace std;

class MotorSpeedHandler : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  const int PWM_PIN = 12;
  int pwmFd;

  void SetMotorSpeed(const std_msgs::msg::Int32& msg) const
  {
    auto queue = lgTxPwm(pwmFd, PWM_PIN, 10000, msg.data, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Setting the speed to: %d%% (%d in queue)", msg.data, queue);
  }

  void OnShutdownCallback()
  {
    lgGpioFree(pwmFd, PWM_PIN);
    lgGpiochipClose(pwmFd);
  }

public:
  MotorSpeedHandler() : Node("motorspeedhandler")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "motorspeed", 10, std::bind(&MotorSpeedHandler::SetMotorSpeed, this, _1));
    
    
    //Initialize pwm
    int pwmFd = lgGpiochipOpen(0);
    if(pwmFd < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open gpio chip 0.  Exiting...");
      rclcpp::shutdown();
    }
    if(lgGpioClaimOutput(pwmFd, 0, PWM_PIN, 0))
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to claim gpio pin %d.  Exiting...", PWM_PIN);
      rclcpp::shutdown();
    }
    rclcpp::on_shutdown(std::bind(&MotorSpeedHandler::OnShutdownCallback, this));
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSpeedHandler>());
  rclcpp::shutdown();
  return 0;
}
