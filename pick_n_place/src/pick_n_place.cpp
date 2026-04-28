#include "pick_n_place/PickNPlace.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<PickNPlace> node_handle = std::make_shared<PickNPlace>();
    
	node_handle->run() ;
		
	rclcpp::shutdown();
	return 0;

}
