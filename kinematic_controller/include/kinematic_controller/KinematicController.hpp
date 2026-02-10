#ifndef KINEMATIC_CONTROLLER__KinematicController_HPP
#define KINEMATIC_CONTROLLER__KinematicController_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>

class KinematicController : public rclcpp::Node
{
public:
  KinematicController();

private:
   /* ===== Callbacks ===== */
  bool readParameters();
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void update();

  /* ---------- ROS Interfaces ---------- */
  
  //Forward Kinematic to know end-effector pose
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  
  // Inverse Kinematic to publish joint desired velocity
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_twist_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_joint_velocity_pub;
  
  //500 Hz
  rclcpp::TimerBase::SharedPtr timer;
  
  /*----------- Messages ----------*/
  geometry_msgs::msg::Pose pose_msg;
  geometry_msgs::msg::Twist twist_msg;
  std_msgs::msg::Float64MultiArray desired_joint_velocity_msg;

  /* ===== Parameters ===== */
  std::string joint_state_topic;
  std::string pose_topic;
  std::string twist_topic;
  std::string urdf_file_name;

  /* ===== Pinocchio ===== */
  pinocchio::Model model;
  pinocchio::Data data;
  int HAND_ID;

  /* ---------- Internal State ---------- */
  Eigen::VectorXd current_joint_state;
  Eigen::VectorXd current_joint_velocity;
  bool has_joint_state;
  
  pinocchio::SE3 fbk_pose;
  Eigen::Vector3d end_eff_pose;
  pinocchio::Motion fbk_twist;
  Eigen::Vector3d end_eff_linear_velocity;
  Eigen::Vector3d end_eff_angular_velocity;
  
  Eigen::Vector3d end_eff_desired_velocity;
  Eigen::MatrixXd J;
  Eigen::MatrixXd Jv;
  Eigen::VectorXd desired_joint_velocity_vector;
};

#endif  // KINEMATIC_CONTROLLER__KinematicController_HPP

