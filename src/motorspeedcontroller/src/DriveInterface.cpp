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

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class DriveInterface : public rclcpp::Node
{
private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorSpeedPublisher_FrontLeft;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorSpeedPublisher_FrontRight;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorSpeedPublisher_BackLeft;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motorSpeedPublisher_BackRight;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocitySubscription;

  void MotorSpeedFromTwist(const geometry_msgs::msg::Twist& msg)
  {
    auto frontLeftSpeed = std_msgs::msg::Int32();
    auto frontRightSpeed = std_msgs::msg::Int32();
    auto backLeftSpeed = std_msgs::msg::Int32();
    auto backRightSpeed = std_msgs::msg::Int32();

    // figure out how the motors have to move to achieve that twist
    // read wheel profile from include file (start with assuming current configuration)

    motorSpeedPublisher_FrontLeft->publish(frontLeftSpeed);
    motorSpeedPublisher_FrontRight->publish(frontRightSpeed);
    motorSpeedPublisher_BackLeft->publish(backLeftSpeed);
    motorSpeedPublisher_BackRight->publish(backRightSpeed);
  }

  // custom message type that gives a point in 3d space to drive to & how much time
  //   void MotorSpeedFromPoint(const Point& msg)
  //   {
  //     motorSpeedPublisher->publish()
  //   }

public:
  DriveInterface() : Node("driveinterface")
  {
    // add launch file that remaps the motor driver subscription topic to these topics for each motor
    motorSpeedPublisher_FrontLeft = this->create_publisher<std_msgs::msg::Int32>("motorspeed_frontleft", 10);
    motorSpeedPublisher_FrontRight = this->create_publisher<std_msgs::msg::Int32>("motorspeed_frontright", 10);
    motorSpeedPublisher_BackLeft = this->create_publisher<std_msgs::msg::Int32>("motorspeed_backleft", 10);
    motorSpeedPublisher_BackRight = this->create_publisher<std_msgs::msg::Int32>("motorspeed_backright", 10);
    velocitySubscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "accelerator", 10, std::bind(&DriveInterface::MotorSpeedFromTwist, this, _1));
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveInterface>());
  rclcpp::shutdown();
  return 0;
}