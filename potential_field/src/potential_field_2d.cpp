#include "potential_field/PotentialField2d.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialField2d>());
    rclcpp::shutdown();
    return 0;
}
