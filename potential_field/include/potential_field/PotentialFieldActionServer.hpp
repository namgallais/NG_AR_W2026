#ifndef POTENTIAL_FIELD__PotentialFieldActionServer_HPP
#define POTENTIAL_FIELD__PotentialFieldActionServer_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "highlevel_interfaces/action/pose_command.hpp"

#include <Eigen/Dense>

using PoseCommandAction = highlevel_interfaces::action::PoseCommand;
using PoseCommandGoalHandle = rclcpp_action::ServerGoalHandle<PoseCommandAction>;

class PotentialFieldActionServer : public rclcpp::Node
{
public:
  PotentialFieldActionServer();

private:
  /* ---------- Callbacks ---------- */
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  void update();
  
  bool readParameters();
  
  /*-----------Action-----------*/
  //Callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, [[maybe_unused]] std::shared_ptr<const PoseCommandAction::Goal> goal) ;
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<PoseCommandGoalHandle> goal_handle) ;
  void handle_accepted(const std::shared_ptr<PoseCommandGoalHandle> goal_handle) ;
  void execute_action(const std::shared_ptr<PoseCommandGoalHandle> goal_handle) ;

  //ROS Interfaces
  rclcpp_action::Server<PoseCommandAction>::SharedPtr action_server_ ;
  std::shared_ptr<PoseCommandAction::Feedback> pose_command_feedback_ ;
  std::shared_ptr<PoseCommandAction::Result> pose_command_response_ ;

  /* ---------- ROS Interfaces ---------- */
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::TimerBase::SharedPtr timer;
  
  /* ---------- Parameters ---------- */
  double k_att;
  int publish_rate;
  double maximum_linear_velocity;
  
  /*----------- Messages ----------*/
  geometry_msgs::msg::Twist twist_msg;
  std_msgs::msg::Bool done_msg;
  
  
  /* ---------- Internal State ---------- */
  Eigen::Vector3d current_pose;
  Eigen::Vector3d target_pose;
  Eigen::Vector3d difference;
  Eigen::Vector3d velocity;
  Eigen::Vector3d ang_vel;

  bool has_pose;
  bool has_target;
  
  double duration;
  double dist_translation;
  double dist_orientation;
  
  Eigen::Quaterniond quat_fbk;
  Eigen::Quaterniond quat_diff;
  Eigen::Quaterniond quat_tar;

  Eigen::AngleAxisd angle_axis;
  
  
  Eigen::Vector3d euler;
  
  double k_att_ang;	
  double sign;
  
  const double dt = 0.002;
};

#endif  // POTENTIAL_FIELD__PotentialFieldActionServer_HPP

