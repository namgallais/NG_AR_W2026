#include "kinematic_controller/KinematicController.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicController>());
    rclcpp::shutdown();
    return 0;
}
