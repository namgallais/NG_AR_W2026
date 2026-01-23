#include "rclcpp/rclcpp.hpp"
#include "keyboard_controller/KeyboardController.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardController>());
  rclcpp::shutdown();
  return 0;
}


