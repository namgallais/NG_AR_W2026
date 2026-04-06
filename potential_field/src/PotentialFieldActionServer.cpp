#include "potential_field/PotentialFieldActionServer.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
PotentialFieldActionServer::PotentialFieldActionServer()
: Node("potential_field_action_server"),
  current_pose(Eigen::Vector3d::Zero()),
  target_pose(Eigen::Vector3d(0.5, 0.2, 0.5)), //A5 requierment
  difference(Eigen::Vector3d::Zero()),
  velocity(Eigen::Vector3d::Zero()),
  has_pose(false),
  has_target(false),
  duration(0.0),
  dist_translation(0.0),
  dist_orientation(0.0),
  quat_fbk(Eigen::Quaterniond::Identity()),
  quat_diff(Eigen::Quaterniond::Identity()),
  quat_tar(Eigen::Quaterniond::Identity()),
  euler(Eigen::Vector3d::Zero())
{

  /************PARAMETERS***************/
  if (!readParameters()) {
        RCLCPP_INFO(this->get_logger(), "Cannot read parameters");
        rclcpp::shutdown() ; // shutdown the node, if it really needs some parameters
  } 
    else {
        RCLCPP_INFO(this->get_logger(), "Successfully read parameters");
  }
  
  
  /****************PUBLISHER AND SUBSCRIBER*****************/
  pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
    "/gen3/feedback/pose",
    10,
    std::bind(&PotentialFieldActionServer::pose_callback, this, std::placeholders::_1)
  );

  twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    "/gen3/reference/twist", 10); //Reference != Feedback
    

  // make the action server	
    action_server_ = rclcpp_action::create_server<PoseCommandAction>(
		this, 						     // node handle
		"/planner/move_pose",			 // name of the action (what you see in ros2 action list)
	std::bind(&PotentialFieldActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    	std::bind(&PotentialFieldActionServer::handle_cancel, this, std::placeholders::_1),
    	std::bind(&PotentialFieldActionServer::handle_accepted, this, std::placeholders::_1)      
	);
	
  pose_command_feedback_ = std::make_shared<PoseCommandAction::Feedback>() ;
  pose_command_response_ = std::make_shared<PoseCommandAction::Result>();

  RCLCPP_INFO(this->get_logger(), "PotentialFieldAction Server ready");
  
  timer = this->create_wall_timer(
    2ms,
    std::bind(&PotentialFieldActionServer::update, this)
  );

  RCLCPP_INFO(this->get_logger(), "Potential field planner 3d started");
}

/* ================= CALLBACKS ================= */

void PotentialFieldActionServer::pose_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  current_pose(0) = msg->position.x;
  current_pose(1) = msg->position.y;
  current_pose(2) = msg->position.z;
  
  quat_fbk.w() = msg->orientation.w;
  quat_fbk.x() = msg->orientation.x;
  quat_fbk.y() = msg->orientation.y;
  quat_fbk.z() = msg->orientation.z;
  
  has_pose = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First pose callback");
}

void PotentialFieldActionServer::update()
{
  
  if (!has_pose) {
    RCLCPP_INFO_ONCE(this->get_logger(), "First update w/o pose");
    return;
  }

  //linear velocity

  difference = target_pose - current_pose;
  velocity = k_att * difference;

  if (velocity.norm() > maximum_linear_velocity) {
    velocity = (velocity / velocity.norm()) * maximum_linear_velocity;
  }
  
  //angular velocity

  quat_diff   = quat_tar * quat_fbk.conjugate();
  angle_axis  = quat_diff ;

  if ( quat_tar.dot(quat_fbk) < 0 ) {
    sign = -1 ;    
  }
  else {
    sign = 1 ;
  }

  ang_vel = sign * k_att_ang * angle_axis.angle() * angle_axis.axis() ;


  twist_msg.linear.x = velocity(0);
  twist_msg.linear.y = velocity(1);
  twist_msg.linear.z = velocity(2);
  twist_msg.angular.x = ang_vel(0);
  twist_msg.angular.y = ang_vel(1);
  twist_msg.angular.z = ang_vel(2);
  done_msg.data = false;

  twist_pub->publish(twist_msg);
  
  RCLCPP_INFO_ONCE(this->get_logger(), "First update with pose");
}

bool PotentialFieldActionServer::readParameters()
{

  this->declare_parameter("k_att", 5.0);
  this->declare_parameter("k_att_ang", 5.0);
  this->declare_parameter("publish_rate", 500);
  this->declare_parameter("maximum_linear_velocity", 1.0);

  /* -------- k_att -------- */  
  if (this->get_parameter("k_att", k_att)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "k_att = " << k_att);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get k_att");
  }
  
  /* -------- k_att_ang -------- */  
  if (this->get_parameter("k_att_ang", k_att_ang)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "k_att_ang = " << k_att_ang);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get k_att_ang");
  }
  
  /* -------- publish_rate -------- */
  if (this->get_parameter("publish_rate", publish_rate)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "publish_rate = " << publish_rate);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get publish_rate");
  }
  
  
  /* -------- maximum_linear_velocity -------- */
  if (this->get_parameter("maximum_linear_velocity", maximum_linear_velocity)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "maximum_linear_velocity = " << maximum_linear_velocity);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get maximum_linear_velocity");
  }

  
  return true;
}


/* ====== this function will be called when a client requests the action ========*/

rclcpp_action::GoalResponse PotentialFieldActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, 
    [[maybe_unused]] std::shared_ptr<const PoseCommandAction::Goal> goal) {

    RCLCPP_INFO(this->get_logger(), "Received goal request for a timed rotation action");
    (void)uuid;
	
  // Check each field individually by name
    if (goal->x < -0.8 || goal->x > 0.8 ||
        goal->y < -0.8 || goal->y > 0.8 ||
        goal->z < 0.01 || goal->z > 0.8 ||
        goal->roll  < -3.14 || goal->roll  > 3.14 ||
        goal->pitch < -3.14 || goal->pitch > 3.14 ||
        goal->yaw   < -3.14 || goal->yaw   > 3.14)
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: outside workspace bounds");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/* ========= this function will be called when goal is accepted ============ */	
void PotentialFieldActionServer::handle_accepted(const std::shared_ptr<PoseCommandGoalHandle> goal_handle) {
        
	RCLCPP_INFO(this->get_logger(), "handle_accepted");
        
        /* std::bind: this it will run execute_action method with goal_handle as the argument 
	   std::thread: start a separate process in the background to execute this function */
	   
	std::thread{
        std::bind(&PotentialFieldActionServer::execute_action, this, std::placeholders::_1), goal_handle
    }.detach();
}


/* where you really process the action*/
void PotentialFieldActionServer::execute_action(
    const std::shared_ptr<PoseCommandGoalHandle> goal_handle)
{
    // 1. Set the new target (update() will immediately start chasing it)
    target_pose(0) = goal_handle->get_goal()->x;
    target_pose(1) = goal_handle->get_goal()->y;
    target_pose(2) = goal_handle->get_goal()->z;
    // (also store target orientation for error calculation)
    
    quat_tar =
  Eigen::AngleAxisd(goal_handle->get_goal()->yaw, Eigen::Vector3d::UnitZ()) 
* Eigen::AngleAxisd(goal_handle->get_goal()->pitch, Eigen::Vector3d::UnitY()) 
* Eigen::AngleAxisd(goal_handle->get_goal()->roll, Eigen::Vector3d::UnitX());

    rclcpp::Time start_time = this->now();

    rclcpp::Rate rate(500);  // 500 Hz, same cadence as your 2ms timer

    while (rclcpp::ok()) {

        duration = (this->now() - start_time).seconds();

        // --- compute errors ---
        dist_translation = (target_pose - current_pose).norm();
        dist_orientation = quat_fbk.angularDistance(quat_tar);

        // --- fill and publish feedback ---
        pose_command_feedback_->distance_translation = dist_translation;
        pose_command_feedback_->distance_orientation = dist_orientation;
        pose_command_feedback_->time_elapsed         = duration;
        goal_handle->publish_feedback(pose_command_feedback_);

        // --- termination conditions ---
        if (dist_translation < 0.05 && dist_orientation < 0.1) {
            pose_command_response_->success = true;
            pose_command_response_->message = "Goal reached";
            goal_handle->succeed(pose_command_response_);
            return;
        }

        if (duration > 5.0) {
            pose_command_response_->success = false;
            pose_command_response_->message = "Timeout";
            goal_handle->abort(pose_command_response_);
            return;
        }

        rate.sleep();  // this controls how fast the loop repeats
    }
}

rclcpp_action::CancelResponse PotentialFieldActionServer::handle_cancel(
    const std::shared_ptr<PoseCommandGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    
    // reset to default pose
    target_pose = Eigen::Vector3d(0.5, 0.2, 0.5);
    
    return rclcpp_action::CancelResponse::ACCEPT;
}
