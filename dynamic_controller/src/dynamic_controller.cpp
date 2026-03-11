#include "dynamic_controller/DynamicController.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicController>());
    rclcpp::shutdown();
    return 0;
}
