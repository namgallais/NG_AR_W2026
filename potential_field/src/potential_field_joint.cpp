#include "potential_field/PotentialFieldJoint.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PotentialFieldJoint>());
    rclcpp::shutdown();
    return 0;
}
