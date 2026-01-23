#ifndef KEYBOARD_CONTROLLER_HPP_
#define KEYBOARD_CONTROLLER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class KeyboardController : public rclcpp::Node
{
public:
  KeyboardController();

private:
  void keyboard_callback(const std_msgs::msg::String::SharedPtr msg);
  void timer_callback();

  std::string last_key_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


#endif  // KEYBOARD_CONTROLLER_HPP_
