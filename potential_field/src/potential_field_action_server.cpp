#include "potential_field/PotentialFieldActionServer.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialFieldActionServer>());
    rclcpp::shutdown();
    return 0;
}
