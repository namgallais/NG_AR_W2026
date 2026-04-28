#ifndef PICK_N_PLACE__PickNPlace_HPP
#define PICK_N_PLACE__PickNPlace_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "highlevel_interfaces/action/pose_command.hpp"
//#include "gripper_controller/gripper_cmd.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"


#include <Eigen/Dense>

using PoseCommandAction = highlevel_interfaces::action::PoseCommand ; 
using PoseCommandGoalHandle = rclcpp_action::ClientGoalHandle<PoseCommandAction>;

using GripperAction = control_msgs::action::ParallelGripperCommand;
using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperAction>;


class PickNPlace : public rclcpp::Node {

public:		
    PickNPlace();
    void run() ;

protected:
    void pose_goal_response_callback (const PoseCommandGoalHandle::SharedPtr& goal_handle) ;
    void pose_feedback_callback (PoseCommandGoalHandle::SharedPtr, const std::shared_ptr<const PoseCommandAction::Feedback> feedback) ;
    void pose_result_callback (const PoseCommandGoalHandle::WrappedResult & result) ;
		
    void gripper_goal_response_callback (const GripperGoalHandle::SharedPtr& goal_handle) ;
    void gripper_feedback_callback (GripperGoalHandle::SharedPtr, const std::shared_ptr<const GripperAction::Feedback> feedback) ;
    void gripper_result_callback (const GripperGoalHandle::WrappedResult & result) ;

private:
    bool readParameters();
    rclcpp::Logger logger_ ; 
		
    //Pose
    rclcpp_action::Client<PoseCommandAction>::SharedPtr pose_client ;
    rclcpp_action::Client<PoseCommandAction>::SendGoalOptions pose_options ;
    //std::shared_ptr<PoseCommandAction::Goal> pose_goal_ ;
    PoseCommandAction::Goal pose_goal ;
    std::string pose_action_name = "/planner/move_pose" ; // or whatever action name

    std::shared_future<PoseCommandGoalHandle::SharedPtr> pose_send_goal_future ;
    rclcpp::FutureReturnCode pose_return_code  ;
    PoseCommandGoalHandle::SharedPtr pose_goal_handle ;

    std::shared_future<PoseCommandGoalHandle::WrappedResult> pose_result_future ;

    bool pose_request_accepted = false ;
    bool pose_action_done;
		
		
    //Gripper 
		
    rclcpp_action::Client<GripperAction>::SharedPtr gripper_client ;
    rclcpp_action::Client<GripperAction>::SendGoalOptions gripper_options ;
    //std::shared_ptr<GripperAction::Goal> gripper_goal_ ;
    GripperAction::Goal gripper_goal ;
    std::string gripper_action_name = "/gripper_controller/gripper_cmd" ; // or whatever action name

    std::shared_future<GripperGoalHandle::SharedPtr> gripper_send_goal_future ;
    rclcpp::FutureReturnCode gripper_return_code  ;
    GripperGoalHandle::SharedPtr gripper_goal_handle ;

    std::shared_future<GripperGoalHandle::WrappedResult> gripper_result_future ;

    bool gripper_request_accepted = false ;
    bool gripper_action_done;
		
    bool done;
    int idx;
    size_t n;
    
    /* ---------- Parameters ---------- */
    std::vector<double> x_vec, y_vec, z_vec, roll_vec, pitch_vec, yaw_vec, grasp_vec;
    std::vector<bool> done_vec;
    std::vector<int64_t> next_vec;
};




#endif  // PICK_N_PLACE__PickNPlace_HPP

