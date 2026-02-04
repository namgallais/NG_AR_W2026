#ifndef POTENTIAL_FIELD__PotentialFieldJoint_HPP
#define POTENTIAL_FIELD__PotentialFieldJoint_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <Eigen/Dense>

class PotentialFieldJoint : public rclcpp::Node
{
public:
  PotentialFieldJoint();
  
private:
  /* ---------- Callbacks ---------- */
  void pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void homing_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void update();
  
  bool readParameters();

  /* ---------- ROS Interfaces ---------- */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service;
  rclcpp::TimerBase::SharedPtr timer;

  /* ---------- Internal State ---------- */
  Eigen::VectorXd current_pose;
  Eigen::VectorXd target_pose;
  Eigen::VectorXd difference;
  Eigen::VectorXd velocity;

  bool has_pose;
  bool has_target;
  int all_done;
  
  /*----------- Messages ----------*/
  std_msgs::msg::Float64MultiArray velocity_msg;
  std_msgs::msg::Bool done_msg;

  /* ---------- Parameters ---------- */
  double k_att;
  double done_threshold;
  Eigen::VectorXd default_joint_position;
  Eigen::VectorXd maximum_joint_velocity;
};

#endif  // POTENTIAL_FIELD__PotentialFieldJoint_HPP

