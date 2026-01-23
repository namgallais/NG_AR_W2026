#include "keyboard_controller/KeyboardController.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

KeyboardController::KeyboardController(): Node("keyboard_controller_node"), last_key_("")
{
  keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>("/keyboard_reader/cmd",10, std::bind(&KeyboardController::keyboard_callback, this, std::placeholders::_1)
  );

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/husky/cmd_vel",10);

  timer_ = this->create_wall_timer(
    10ms,   // 100 Hz
    std::bind(&KeyboardController::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Keyboard controller started (100 Hz)");
}

void KeyboardController::keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
{
  last_key_ = msg->data;
}

void KeyboardController::timer_callback()
{
  geometry_msgs::msg::TwistStamped twistS;

  if (last_key_ == "i") {
    twistS.twist.linear.x = 0.5;
    twistS.twist.angular.z = 0.0;
  }
  else if (last_key_ == "u") {
    twistS.twist.linear.x = 0.5;
    twistS.twist.angular.z = 0.5;
  }
  else if (last_key_ == "o") {
    twistS.twist.linear.x = 0.5;
    twistS.twist.angular.z = -0.5;
  }
  else {
    twistS.twist.linear.x = 0.0;
    twistS.twist.angular.z = 0.0;
  }

  cmd_vel_publisher_->publish(twistS);
}

