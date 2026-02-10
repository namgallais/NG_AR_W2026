#include "potential_field/PotentialFieldTask.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
PotentialFieldTask::PotentialFieldTask()
: Node("potential_field_task"),
  current_pose(Eigen::Vector3d::Zero()),
  target_pose(Eigen::Vector3d::Zero()),
  difference(Eigen::Vector3d::Zero()),
  velocity(Eigen::Vector3d::Zero()),
  has_pose(false),
  has_target(false)
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
    pose_topic,
    10,
    std::bind(&PotentialFieldTask::pose_callback, this, std::placeholders::_1)
  );

  twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    twist_topic, 10);

  done_pub = this->create_publisher<std_msgs::msg::Bool>(
    "/planner/done", 10);

  move_to_service =
    this->create_service<highlevel_interfaces::srv::Move3d>(
      "/planner/move_to",
      std::bind(
        &PotentialFieldTask::move_to_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

  timer = this->create_wall_timer(
    2ms,
    std::bind(&PotentialFieldTask::update, this)
  );

  RCLCPP_INFO(this->get_logger(), "Potential field planner 3d started");
}

/* ================= CALLBACKS ================= */

void PotentialFieldTask::pose_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  current_pose(0) = msg->position.x;
  current_pose(1) = msg->position.y;
  current_pose(2) = msg->position.z;
  has_pose = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First pose callback");
}

void PotentialFieldTask::move_to_callback(
  const std::shared_ptr<highlevel_interfaces::srv::Move3d::Request> request,
  std::shared_ptr<highlevel_interfaces::srv::Move3d::Response> response)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "First move_to_3d service callback");

  if (!has_pose) {
    response->success = false;
    return;
  }

  target_pose(0) = request->x;
  target_pose(1) = request->y;
  target_pose(2) = request->z;
  has_target = true;

  response->success = true;
}

void PotentialFieldTask::update()
{
  
  if (!has_pose || !has_target) {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    done_msg.data = true;

    twist_pub->publish(twist_msg);
    done_pub->publish(done_msg);

    RCLCPP_INFO_ONCE(this->get_logger(), "First update");
    return;
  }

  difference = target_pose - current_pose;

  if (difference.norm() < done_threshold) {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    done_msg.data = true;

    twist_pub->publish(twist_msg);
    done_pub->publish(done_msg);
    return;
  }

  velocity = k_att * difference;

  if (velocity.norm() > maximum_linear_velocity) {
    velocity = (velocity / velocity.norm()) * maximum_linear_velocity;
  }

  twist_msg.linear.x = velocity(0);
  twist_msg.linear.y = velocity(1);
  twist_msg.linear.z = velocity(2);
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  done_msg.data = false;

  twist_pub->publish(twist_msg);
  done_pub->publish(done_msg);
}

bool PotentialFieldTask::readParameters()
{

  this->declare_parameter("k_att", 5.0);
  this->declare_parameter("done_threshold", 0.05);
  this->declare_parameter("publish_rate", 500);
  this->declare_parameter("maximum_linear_velocity", 1.0);
  this->declare_parameter<std::string>("pose_topic", "/gen3/feedback/pose"); // Feedback
  this->declare_parameter<std::string>("twist_topic", "/gen3/reference/twist"); //Reference != Feedback

  /* -------- k_att -------- */  
  if (this->get_parameter("k_att", k_att)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "k_att = " << k_att);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get k_att");
  }


  /* -------- done_threshold -------- */
  if (this->get_parameter("done_threshold", done_threshold)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "done_threshold = " << done_threshold);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get done_threshold");
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
  

  /* -------- pose_topic -------- */  
  if (this->get_parameter("pose_topic", pose_topic)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "pose_topic = " << pose_topic);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get pose_topic");
    return false;
  }
  
  
   /* -------- twist_topic -------- */  
  if (this->get_parameter("twist_topic", twist_topic)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "twist_topic = " << twist_topic);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get twist_topic");
    return false;
  }

  return true;
}
