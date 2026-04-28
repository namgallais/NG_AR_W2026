#include "pick_n_place/PickNPlace.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickNPlace>());
    rclcpp::shutdown();
    return 0;
}
