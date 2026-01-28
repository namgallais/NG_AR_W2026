#include "potential_field/PotentialField.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialField>());
    rclcpp::shutdown();
    return 0;
}
