#include "potential_field/PotentialFieldJoint.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
PotentialFieldJoint::PotentialFieldJoint()
: Node("potential_field_joint"),
  current_pose(Eigen::VectorXd::Zero(7)),
  target_pose(Eigen::VectorXd::Zero(7)),
  difference(Eigen::VectorXd::Zero(7)),
  velocity(Eigen::VectorXd::Zero(7)),
  has_pose(false),
  has_target(false)
{
  
  /************PARAMETERS***************/
  
  this->declare_parameter<double>("k_att", 0.0);
  if (!this->get_parameter("k_att", k_att)) {
  RCLCPP_ERROR(rclcpp::get_logger("PotentialFieldJoint"), "parameter k_att not found");
  rclcpp::shutdown();
  }
  
  this->declare_parameter<double>("done_threshold", 0.0);
  if (!this->get_parameter("done_threshold", done_threshold)) {
  RCLCPP_ERROR(rclcpp::get_logger("PotentialFieldJoint"), "parameter done_threshold not found");
  rclcpp::shutdown();
  }
  
  std::vector<double> param_list; //Vector parameters
  int param_size;
  
  this->declare_parameter<std::vector<double>>("default_joint_position", std::vector<double>{});
  if (!this->get_parameter("default_joint_position", param_list)) {
  RCLCPP_ERROR(rclcpp::get_logger("PotentialFieldJoint"), "parameter default_joint_position not found");
  rclcpp::shutdown();
  }
  param_size = param_list.size();
  default_joint_position = Eigen::VectorXd::Zero(param_size);
  for (int i = 0; i < param_size; ++i) {
  default_joint_position(i) = param_list.at(i);
  }
  
  this->declare_parameter<std::vector<double>>("maximum_joint_velocity", std::vector<double>{});
  if (!this->get_parameter("maximum_joint_velocity", param_list)) {
  RCLCPP_ERROR(rclcpp::get_logger("PotentialFieldJoint"), "parameter maximum_joint_velocity not found");
  rclcpp::shutdown();
  }
  param_size = param_list.size();
  maximum_joint_velocity = Eigen::VectorXd::Zero(param_size);
  for (int i = 0; i < param_size; ++i) {
  maximum_joint_velocity(i) = param_list.at(i);
  }
  

  /****************PUBLISHER AND SUBSCRIBER*****************/
  pose_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&PotentialFieldJoint::pose_callback, this, std::placeholders::_1)
  );

  velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/planner/joint_velocity", 10);

  done_pub = this->create_publisher<std_msgs::msg::Bool>(
    "/planner/done", 10);
    
  homing_service = this->create_service<std_srvs::srv::Trigger>(
      "/planner/homing",
      std::bind(
        &PotentialFieldJoint::homing_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

  timer = this->create_wall_timer(
    2ms,
    std::bind(&PotentialFieldJoint::update, this)
  );

  RCLCPP_INFO(this->get_logger(), "Potential field planner started");
}

/* ================= CALLBACKS ================= */

void PotentialFieldJoint::pose_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_pose = Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size());
  has_pose = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First pose callback");
}

void PotentialFieldJoint::homing_callback(
const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "First homing_to service callback");
  
  target_pose = default_joint_position;
  has_target = true;
  
  response->success = true;
  response->message = "Homing started";
}

void PotentialFieldJoint::update()
{
  std_msgs::msg::Float64MultiArray velocity_msg;
  std_msgs::msg::Bool done_msg;

  velocity_msg.data.resize(7);

  if (!has_pose || !has_target) {
    velocity_msg.data.assign(7, 0.0);
    done_msg.data = true;

    velocity_pub->publish(velocity_msg);
    done_pub->publish(done_msg);

    RCLCPP_INFO_ONCE(this->get_logger(), "First update");
    return;
  }

  difference = target_pose - current_pose;
  velocity = k_att * difference;
  int all_done = 0;
  
  for(int i = 0; i < difference.size(); i++) {
    if (difference(i) < done_threshold) {
      velocity_msg.data[i] = 0.0;
      all_done+=1;

    }
    
    if (velocity(i) > maximum_joint_velocity(i)) {
        velocity(i) = maximum_joint_velocity(i);
    }
    
    velocity_msg.data[i] = velocity(i);
    
  }
  
  if(all_done == difference.size()){
      done_msg.data = true;
      velocity_pub->publish(velocity_msg);
      done_pub->publish(done_msg);
      return;
  }
  
  done_msg.data = false;

  velocity_pub->publish(velocity_msg);
  done_pub->publish(done_msg);
}

