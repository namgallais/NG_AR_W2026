#ifndef POTENTIAL_FIELD__POTENTIALFIELD_HPP
#define POTENTIAL_FIELD__POTENTIALFIELD_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "highlevel_interfaces/srv/move2d.hpp"
#include <Eigen/Dense>

class PotentialField : public rclcpp::Node
{
public:
  PotentialField();

private:
  /* ---------- Callbacks ---------- */
  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  void move_to_callback(
    const std::shared_ptr<highlevel_interfaces::srv::Move2d::Request> request,
    std::shared_ptr<highlevel_interfaces::srv::Move2d::Response> response);

  void update();

  /* ---------- ROS Interfaces ---------- */
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub;
  rclcpp::Service<highlevel_interfaces::srv::Move2d>::SharedPtr move_to_service;
  rclcpp::TimerBase::SharedPtr timer;

  /* ---------- Internal State ---------- */
  Eigen::Vector2d current_pose;
  Eigen::Vector2d target_pose;
  Eigen::Vector2d difference;
  Eigen::Vector2d velocity;

  bool has_pose;
  bool has_target;

  /* -------- Attractive force -------- */
  const double k_att = 5.0;
  const double max_vel = 1.0;
};

#endif  // POTENTIAL_FIELD__POTENTIALFIELD_HPP

