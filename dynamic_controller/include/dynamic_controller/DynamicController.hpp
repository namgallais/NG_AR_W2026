#ifndef DYNAMIC_CONTROLLER__DynamicController_HPP
#define DYNAMIC_CONTROLLER__DynamicController_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "highlevel_interfaces/srv/set_kd.hpp"
#include <Eigen/Dense>

class DynamicController : public rclcpp::Node
{
public:
  DynamicController();

private:
   /* ===== Callbacks ===== */

  bool readParameters();

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void ref_position_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void ref_velocity_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void ref_pose_Callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  void ref_twist_Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  void set_linear_kd_callback(
  const std::shared_ptr<highlevel_interfaces::srv::SetKD::Request> request,
  std::shared_ptr<highlevel_interfaces::srv::SetKD::Response> response);

void set_joint_kd_callback(
  const std::shared_ptr<highlevel_interfaces::srv::SetKD::Request> request,
  std::shared_ptr<highlevel_interfaces::srv::SetKD::Response> response);

  void update();

  /* ---------- ROS Interfaces ---------- */
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_position_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_velocity_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_twist_sub;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr fbk_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr fbk_twist_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_joint_effort_pub;
  
  //Services
  rclcpp::Service<highlevel_interfaces::srv::SetKD>::SharedPtr set_linear_kd_service;
  rclcpp::Service<highlevel_interfaces::srv::SetKD>::SharedPtr set_joint_kd_service;
  
  
  //500 Hz
  rclcpp::TimerBase::SharedPtr timer;
  
  /*----------- Messages ----------*/
  geometry_msgs::msg::Pose pose_msg;
  geometry_msgs::msg::Twist twist_msg;
  std_msgs::msg::Float64MultiArray desired_joint_effort_msg;

  /* ===== Parameters ===== */
  std::string urdf_file_name;

  bool with_redundancy;

  double linear_k;
  double linear_d;
  double joint_k;
  double joint_d;

  /* ===== Pinocchio ===== */
  pinocchio::Model model;
  pinocchio::Data data;
  int hand_id;
  int dim_joints;

/* ========= Internal State ========= */
  /* ---------- Robot State ---------- */

  Eigen::VectorXd fbk_q_position;
  Eigen::VectorXd fbk_q_velocity;

  Eigen::Vector3d fbk_end_eff_pose;
  Eigen::VectorXd fbk_twist;
  
  Eigen::Vector3d x_fbk;
  Eigen::Vector3d dx_fbk;

/* ---------- References ---------- */

  Eigen::VectorXd ref_q_position;
  Eigen::VectorXd ref_q_velocity;

  Eigen::Vector3d x_ref;
  Eigen::Vector3d dx_ref;

/* ---------- Dynamics ---------- */

  Eigen::MatrixXd J;
  Eigen::MatrixXd J_dot;

  Eigen::MatrixXd Jv;
  Eigen::MatrixXd Jv_dot;

  Eigen::MatrixXd Jv_pinv;

  Eigen::MatrixXd M;
  Eigen::VectorXd h;
  
  Eigen::Matrix3d Lambda;
  Eigen::Vector3d eta;

  Eigen::MatrixXd P;
  Eigen::MatrixXd I;
  
  Eigen::Vector3d ddx_cmd;

  Eigen::VectorXd tau_cmd;
  
  
/* ---------- Has_xxx ---------- */

  bool has_joint_state;
  bool has_ref_position;
  bool has_ref_velocity;
  bool has_ref_pose;
  bool has_ref_twist;

};

#endif  // DYNAMIC_CONTROLLER__DynamicController_HPP

