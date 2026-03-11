#ifndef POTENTIAL_FIELD__PotentialFieldTask_HPP
#define POTENTIAL_FIELD__PotentialFieldTask_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "highlevel_interfaces/srv/move3d.hpp"
#include <Eigen/Dense>

class PotentialFieldTask : public rclcpp::Node
{
public:
  PotentialFieldTask();

private:
  /* ---------- Callbacks ---------- */
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  void move_to_callback(
    const std::shared_ptr<highlevel_interfaces::srv::Move3d::Request> request,
    std::shared_ptr<highlevel_interfaces::srv::Move3d::Response> response);

  void update();
  
  bool readParameters();

  /* ---------- ROS Interfaces ---------- */
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ref_pose_pub;
  rclcpp::Service<highlevel_interfaces::srv::Move3d>::SharedPtr move_to_service;
  rclcpp::TimerBase::SharedPtr timer;
  
  /* ---------- Parameters ---------- */
  double k_att;
  int publish_rate;
  double maximum_linear_velocity;
  std::string pose_topic;
  std::string twist_topic;
  
  /*----------- Messages ----------*/
  geometry_msgs::msg::Twist twist_msg;
  geometry_msgs::msg::Pose ref_pose_msg;
  std_msgs::msg::Bool done_msg;
  
  
  /* ---------- Internal State ---------- */
  Eigen::Vector3d current_pose;
  Eigen::Vector3d target_pose;
  Eigen::Vector3d difference;
  Eigen::Vector3d velocity;
  Eigen::Vector3d ref_pose;

  bool has_pose;
  bool has_target;
  
  const double dt = 0.002;
};

#endif  // POTENTIAL_FIELD__PotentialFieldTask_HPP

