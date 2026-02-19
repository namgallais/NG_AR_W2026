#include "potential_field/PotentialFieldJoint.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
PotentialFieldJoint::PotentialFieldJoint()
: Node("potential_field_joint"),
  current_pose(Eigen::VectorXd::Zero(7)),
  difference(Eigen::VectorXd::Zero(7)),
  velocity(Eigen::VectorXd::Zero(7)),
  has_pose(false)
{
  ref_velocity_msg.data.resize(7);
  ref_velocity_msg.data.assign(7, 0.0);
  
  /************PARAMETERS***************/
  
  
  if (!readParameters()) {
        RCLCPP_INFO(this->get_logger(), "Cannot read parameters");
        rclcpp::shutdown() ; // shutdown the node, if it really needs some parameters
  } 
    else {
        RCLCPP_INFO(this->get_logger(), "Successfully read parameters");
  }
  

  /****************PUBLISHER AND SUBSCRIBER*****************/
  pose_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&PotentialFieldJoint::pose_callback, this, std::placeholders::_1)
  );
    
  ref_velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
  "/gen3/reference/velocity", 10);
    
    
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

void PotentialFieldJoint::update()
{

  RCLCPP_INFO_ONCE(this->get_logger(), "First update");

  if (!has_pose) return;
  
  difference = default_joint_position - current_pose;
  velocity = k_att * difference;
  
  for(int i = 0; i < 7; i++) {
  
    if (velocity(i) > maximum_joint_velocity(i)) {
        velocity(i) = maximum_joint_velocity(i);
    }
    if (velocity(i) < -maximum_joint_velocity(i)) {
        velocity(i) = -maximum_joint_velocity(i);
    }
    
    ref_velocity_msg.data[i] = velocity(i);
    
  }
  
  ref_velocity_pub->publish(ref_velocity_msg);
  
}

/******************* READ PARAMETERS *************/

bool PotentialFieldJoint::readParameters()
{
  std::vector<double> param_list;

  this->declare_parameter("k_att", 5.0);
  this->declare_parameter("default_joint_position", std::vector<double>{0.0, 0.6, 0.0, 0.6, 0.0, 0.6, 0.0});
  this->declare_parameter("maximum_joint_velocity", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  
  /* -------- k_att -------- */  
  if (this->get_parameter("k_att", k_att)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "k_att = " << k_att);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get k_att");
  }


  /* -------- default_joint_position -------- */
  if (!this->get_parameter("default_joint_position", param_list)) {
        RCLCPP_WARN(this->get_logger(), "Cannot read default_joint_position");
    }
    else {
        default_joint_position = Eigen::VectorXd::Zero(param_list.size());
        for (int i = 0; i < (int)param_list.size(); i++) {
            default_joint_position(i) = param_list.at(i);
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
        "default_joint_position: " << default_joint_position.transpose());


  /* -------- maximum_joint_velocity -------- */
  if (!this->get_parameter("maximum_joint_velocity", param_list)) {
        RCLCPP_WARN(this->get_logger(), "Cannot read maximum_joint_velocity");
    }
    else {
        maximum_joint_velocity = Eigen::VectorXd::Zero(param_list.size());
        for (int i = 0; i < (int)param_list.size(); i++) {
            maximum_joint_velocity(i) = param_list.at(i);
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
        "maximum_joint_velocity: " << maximum_joint_velocity.transpose());

  return true;
}

