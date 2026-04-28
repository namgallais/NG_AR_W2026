#include "pick_n_place/PickNPlace.hpp"

#include <cmath>
#include <chrono>

PickNPlace::PickNPlace() 
: rclcpp::Node("pick_n_place"), 
  logger_(this->get_logger()),
  done(false),
  idx(0),
  n(0)
{
        if (!readParameters()) {
          RCLCPP_INFO(this->get_logger(), "Cannot read parameters");
          rclcpp::shutdown() ; // shutdown the node, if it really needs some parameters
        } 
        else {
          RCLCPP_INFO(this->get_logger(), "Successfully read parameters");
        }
        
        n = x_vec.size();
        if (y_vec.size() != n || z_vec.size() != n || roll_vec.size() != n || pitch_vec.size() != n || yaw_vec.size() != n || grasp_vec.size() != n || next_vec.size() != n) {
          RCLCPP_ERROR(logger_, "All sequence vectors must have the same size!");
          rclcpp::shutdown();
        }

	// set up action client
        pose_client = rclcpp_action::create_client<PoseCommandAction>(this, pose_action_name) ;
        
        gripper_client = rclcpp_action::create_client<GripperAction>(this, gripper_action_name) ;

	// set up optinal feedback
	pose_options = rclcpp_action::Client<PoseCommandAction>::SendGoalOptions{};	
	pose_options.feedback_callback = std::bind(&PickNPlace::pose_feedback_callback, this, std::placeholders::_1, std::placeholders::_2); 
	pose_options.goal_response_callback = std::bind(&PickNPlace::pose_goal_response_callback, this, std::placeholders::_1); 
	pose_options.result_callback = std::bind(&PickNPlace::pose_result_callback, this, std::placeholders::_1); 
	
	
	gripper_options = rclcpp_action::Client<GripperAction>::SendGoalOptions{};	
	gripper_options.feedback_callback = std::bind(&PickNPlace::gripper_feedback_callback, this, std::placeholders::_1, std::placeholders::_2); 
	gripper_options.goal_response_callback = std::bind(&PickNPlace::gripper_goal_response_callback, this, std::placeholders::_1); 
	gripper_options.result_callback = std::bind(&PickNPlace::gripper_result_callback, this, std::placeholders::_1); 
	

	// the action goal
	gripper_goal.command.name  = {"right_finger_bottom_joint"};

	// wait for the action server to be ready. time out after 10 seconds
	if ( !pose_client->wait_for_action_server(std::chrono::seconds(10))) {		
    	RCLCPP_ERROR(logger_, "Pose action server is not available");
    	rclcpp::shutdown();    	
  	}
  	// wait for the action server to be ready. time out after 10 seconds
	if ( !gripper_client->wait_for_action_server(std::chrono::seconds(10))) {		
    	RCLCPP_ERROR(logger_, "Gripper action server is not available");
    	rclcpp::shutdown();    	
  	}

	RCLCPP_INFO_STREAM(logger_, "Successfully launched node.") ;
}

void PickNPlace::run() {
	while(!done) {
	        if (idx < 0 || idx >= (int)n) {
                  RCLCPP_ERROR(logger_, "parameters index out of bounds");
                  done = true;
                  return;
                }
                
                gripper_action_done = false;
                pose_action_done = false;
	
	        pose_goal.x     =     x_vec[idx];
                pose_goal.y     =     y_vec[idx];
                pose_goal.z     =     z_vec[idx];
                pose_goal.roll  =  roll_vec[idx];
                pose_goal.pitch = pitch_vec[idx];
                pose_goal.yaw   =   yaw_vec[idx];
      
                gripper_goal.command.position = {grasp_vec[idx]};
        
                done  =  done_vec[idx];
                
                /*====First the Gripper====*/
                
                
                // send the action request
                gripper_send_goal_future = gripper_client -> async_send_goal(gripper_goal, gripper_options) ;
                
                // spin until the action server finishes or abort. time-out after 5 seconds
                gripper_return_code = rclcpp::spin_until_future_complete(this->get_node_base_interface(), gripper_send_goal_future, std::chrono::seconds(5)) ;
                
                // double check the action request is valid
                if ( gripper_return_code != rclcpp::FutureReturnCode::SUCCESS ) {
 			RCLCPP_ERROR(logger_, "Gripper goal was rejected by server");
			return ;
		}
                
                // the action is triggered. let's wait for the results
                gripper_goal_handle = gripper_send_goal_future.get();
		gripper_result_future = gripper_client->async_get_result(gripper_goal_handle);
		
                // spin until action server finishes
		// when its done, result_callback will be called automatically
                rclcpp::spin_until_future_complete(this->get_node_base_interface(), gripper_result_future);


                /*====Then the Pose====*/
                
                
		// send the action request
		pose_send_goal_future = pose_client -> async_send_goal(pose_goal, pose_options) ;
		
		// spin until the action server finishes or abort. time-out after 5 seconds
		pose_return_code = rclcpp::spin_until_future_complete(this->get_node_base_interface(), pose_send_goal_future, std::chrono::seconds(5)) ;
		
		// double check the action request is valid
		if ( pose_return_code != rclcpp::FutureReturnCode::SUCCESS ) {
 			RCLCPP_ERROR(logger_, " Pose goal was rejected by server");
			return ;
		}

		// the action is triggered. let's wait for the results
		pose_goal_handle = pose_send_goal_future.get();
		pose_result_future = pose_client->async_get_result(pose_goal_handle);

		// spin until action server finishes
		// when its done, result_callback will be called automatically
		rclcpp::spin_until_future_complete(this->get_node_base_interface(), pose_result_future);
		
		RCLCPP_INFO_STREAM(logger_, "\n\nSent pose and gripper action goal idx " << idx ) ;
		
		idx = next_vec[idx];
	}

	RCLCPP_INFO_STREAM(logger_, "All actions done") ;	
}

/*
	response of the action request
*/
void PickNPlace::pose_goal_response_callback (const PoseCommandGoalHandle::SharedPtr& pose_goal_handle) {

	if (!pose_goal_handle) {
		RCLCPP_ERROR(logger_, "pose_goal_response_callback: Goal was rejected by server");
		pose_request_accepted = false ;	
	} 
	else {
		RCLCPP_INFO(logger_, "pose_goal_response_callback: Goal accepted by server, waiting for result");
		pose_request_accepted = true ;
	}
}

/*
	the call back function for the feedback
	the first argument is unused, so no parameter name
*/
void PickNPlace::pose_feedback_callback (PoseCommandGoalHandle::SharedPtr, const std::shared_ptr<const PoseCommandAction::Feedback> feedback) {
	RCLCPP_INFO_STREAM(logger_, "pose_feedback_callback: distance_translation  " << feedback->distance_translation);
	RCLCPP_INFO_STREAM(logger_, "pose_feedback_callback: distance_orientation " << feedback->distance_orientation);
}

/*
	when the action server is done, this function is called
*/
void PickNPlace::pose_result_callback (const PoseCommandGoalHandle::WrappedResult& result) {

	pose_action_done = true ;
	switch (result.code) {

		case rclcpp_action::ResultCode::SUCCEEDED:

			RCLCPP_INFO(logger_, "pose_result_callback: Action was successful");
			// check if the result is valid before accessing its fields
			if (result.result) {
				RCLCPP_INFO_STREAM(logger_, result.result->message);					
			}
			break;
		
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(logger_, "pose_result_callback: Goal was aborted");
			break;
		
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(logger_, "pose_result_callback: Goal was canceled");
			break;
		
		default:
			RCLCPP_ERROR(logger_, "pose_result_callback: Unknown result code");
			break;
	}
}



/*===================================================Same with Gripper================================*/









void PickNPlace::gripper_goal_response_callback (const GripperGoalHandle::SharedPtr& gripper_goal_handle) {

	if (!gripper_goal_handle) {
		RCLCPP_ERROR(logger_, "gripper_goal_response_callback: Goal was rejected by server");
		gripper_request_accepted = false ;	
	} 
	else {
		RCLCPP_INFO(logger_, "gripper_goal_response_callback: Goal accepted by server, waiting for result");
		gripper_request_accepted = true ;
	}
}

/*
	the call back function for the feedback
	the first argument is unused, so no parameter name
*/
void PickNPlace::gripper_feedback_callback (GripperGoalHandle::SharedPtr, const std::shared_ptr<const GripperAction::Feedback> feedback) {
	// current position of the finger while moving
    if (!feedback->state.position.empty()) {
        RCLCPP_INFO_STREAM(logger_, 
            "gripper moving... current position = " << feedback->state.position[0]);
    }
    
    //current effort to know if we're grasping
    if (!feedback->state.effort.empty()) {
        RCLCPP_INFO_STREAM(logger_, 
            "gripper current effort = " << feedback->state.effort[0]);
    }
}

/*
	when the action server is done, this function is called
*/
void PickNPlace::gripper_result_callback (const GripperGoalHandle::WrappedResult& result) {

	gripper_action_done = true ;
	switch (result.code) {

		case rclcpp_action::ResultCode::SUCCEEDED:

			RCLCPP_INFO(logger_, "gripper_result_callback: Action was successful");
			// check if the result is valid before accessing its fields
			if (result.result) {
                            RCLCPP_INFO_STREAM(logger_, 
                            "reached_goal=" << (int)result.result->reached_goal <<
                            "  stalled="    << (int)result.result->stalled);
                        }
			break;
		
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(logger_, "gripper_result_callback: Goal was aborted");
			break;
		
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(logger_, "gripper_result_callback: Goal was canceled");
			break;
		
		default:
			RCLCPP_ERROR(logger_, "gripper_result_callback: Unknown result code");
			break;
	}
}

template<typename T>
std::string vec_to_str(const std::vector<T>& v) {
    std::string s = "[";
    for (int i = 0; i < (int)v.size(); i++) {
        s += std::to_string(v[i]);
        if (i < (int)v.size() - 1) s += ", ";
    }
    return s + "]";
}

bool PickNPlace::readParameters()
{

   // ── declare with defaults ────────────────────────────────────
    this->declare_parameter("x_vec",     rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("y_vec",     rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("z_vec",     rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("roll_vec",  rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("pitch_vec", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("yaw_vec",   rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("grasp_vec", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("done_vec",  rclcpp::PARAMETER_BOOL_ARRAY);
    this->declare_parameter("next_vec",  rclcpp::PARAMETER_INTEGER_ARRAY);

    // ── read each parameter, catch failure ───────────────────────
    try { x_vec     = this->get_parameter("x_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read x_vec");     return false; }

    try { y_vec     = this->get_parameter("y_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read y_vec");     return false; }

    try { z_vec     = this->get_parameter("z_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read z_vec");     return false; }

    try { roll_vec  = this->get_parameter("roll_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read roll_vec");  return false; }

    try { pitch_vec = this->get_parameter("pitch_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read pitch_vec"); return false; }

    try { yaw_vec   = this->get_parameter("yaw_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read yaw_vec");   return false; }

    try { grasp_vec = this->get_parameter("grasp_vec").as_double_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read grasp_vec"); return false; }

    try { done_vec  = this->get_parameter("done_vec").as_bool_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read done_vec");  return false; }

    try { next_vec  = this->get_parameter("next_vec").as_integer_array(); }
    catch (...) { RCLCPP_ERROR(logger_, "Failed to read next_vec");  return false; }

    RCLCPP_INFO_STREAM(logger_,
            "  x="     << vec_to_str(x_vec)
            << "\n  y="     << vec_to_str(y_vec)
            << "\n  z="     << vec_to_str(z_vec)
            << "\n  roll="  << vec_to_str(roll_vec)
            << "\n  pitch=" << vec_to_str(pitch_vec)
            << "\n  yaw="   << vec_to_str(yaw_vec)
            << "\n  grasp=" << vec_to_str(grasp_vec)
            << "\n  done="  << vec_to_str(done_vec)
            << "\n  next="  << vec_to_str(next_vec));

    return true;
}
