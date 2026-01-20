#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class KeyboardControllerNode : public rclcpp::Node
{
public:
  KeyboardControllerNode()
  : Node("keyboard_controller_node")
  {
    // Subscriber: keyboard input
    keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/keyboard_reader/cmd",
      10,
      std::bind(&KeyboardControllerNode::keyboard_callback, this, std::placeholders::_1)
    );

    // Publisher: Husky velocity command
    cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/husky/cmd_vel", 10);

    // Timer: 100 Hz (same style as hello_topic)
    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&KeyboardControllerNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Keyboard controller node started (100 Hz)");
  }

private:
  // Store last received keyboard command
  std::string last_key_ = "";

  void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_key_ = msg->data;
  }

  void timer_callback()
  {
    auto twist = geometry_msgs::msg::Twist();

    if (last_key_ == "i") {
      twist.linear.x = 0.5;
      twist.angular.z = 0.0;
    }
    else if (last_key_ == "u") {
      twist.linear.x = 0.5;
      twist.angular.z = 0.5;
    }
    else if (last_key_ == "o") {
      twist.linear.x = 0.5;
      twist.angular.z = -0.5;
    }
    else {
      // Zero velocity
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    }

    cmd_vel_publisher_->publish(twist);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardControllerNode>());
  rclcpp::shutdown();
  return 0;
}

