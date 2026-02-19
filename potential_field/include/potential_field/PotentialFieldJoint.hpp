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

  void update();
  
  bool readParameters();
  
  void ref_velocity_compute();

  /* ---------- ROS Interfaces ---------- */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_velocity_pub;
  
  rclcpp::TimerBase::SharedPtr timer;

  /* ---------- Internal State ---------- */
  Eigen::VectorXd current_pose;
  Eigen::VectorXd difference;
  Eigen::VectorXd velocity;

  bool has_pose;

  /*----------- Messages ----------*/
  std_msgs::msg::Float64MultiArray ref_velocity_msg;

  /* ---------- Parameters ---------- */
  double k_att;
  Eigen::VectorXd default_joint_position;
  Eigen::VectorXd maximum_joint_velocity;
};

#endif  // POTENTIAL_FIELD__PotentialFieldJoint_HPP

