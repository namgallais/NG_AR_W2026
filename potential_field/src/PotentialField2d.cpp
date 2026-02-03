#include "potential_field/PotentialField2d.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
PotentialField2d::PotentialField2d()
: Node("potential_field_2d"),
  current_pose(Eigen::Vector2d::Zero()),
  target_pose(Eigen::Vector2d::Zero()),
  difference(Eigen::Vector2d::Zero()),
  velocity(Eigen::Vector2d::Zero()),
  has_pose(false),
  has_target(false)
{
  pose_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/husky/platform/pose",
    10,
    std::bind(&PotentialField2d::pose_callback, this, std::placeholders::_1)
  );

  velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/planner/velocity", 10);

  done_pub = this->create_publisher<std_msgs::msg::Bool>(
    "/planner/done", 10);

  move_to_service =
    this->create_service<highlevel_interfaces::srv::Move2d>(
      "/planner/move_to",
      std::bind(
        &PotentialField2d::move_to_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

  timer = this->create_wall_timer(
    2ms,
    std::bind(&PotentialField2d::update, this)
  );

  RCLCPP_INFO(this->get_logger(), "Potential field planner started");
}

/* ================= CALLBACKS ================= */

void PotentialField2d::pose_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  current_pose(0) = msg->poses[5].position.x;
  current_pose(1) = msg->poses[5].position.y;
  has_pose = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First pose callback");
}

void PotentialField2d::move_to_callback(
  const std::shared_ptr<highlevel_interfaces::srv::Move2d::Request> request,
  std::shared_ptr<highlevel_interfaces::srv::Move2d::Response> response)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "First move_to service callback");

  if (!has_pose) {
    response->success = false;
    return;
  }

  target_pose(0) = request->x;
  target_pose(1) = request->y;
  has_target = true;

  response->success = true;
}

void PotentialField2d::update()
{
  std_msgs::msg::Float64MultiArray velocity_msg;
  std_msgs::msg::Bool done_msg;

  velocity_msg.data.resize(2);

  if (!has_pose || !has_target) {
    velocity_msg.data[0] = 0.0;
    velocity_msg.data[1] = 0.0;
    done_msg.data = true;

    velocity_pub->publish(velocity_msg);
    done_pub->publish(done_msg);

    RCLCPP_INFO_ONCE(this->get_logger(), "First update");
    return;
  }

  difference = target_pose - current_pose;

  if (difference.norm() < 0.1) {
    velocity_msg.data[0] = 0.0;
    velocity_msg.data[1] = 0.0;
    done_msg.data = true;

    velocity_pub->publish(velocity_msg);
    done_pub->publish(done_msg);
    return;
  }

  velocity = k_att * difference;

  if (velocity.norm() > max_vel) {
    velocity = (velocity / velocity.norm()) * max_vel;
  }

  velocity_msg.data[0] = velocity(0);
  velocity_msg.data[1] = velocity(1);
  done_msg.data = false;

  velocity_pub->publish(velocity_msg);
  done_pub->publish(done_msg);
}

