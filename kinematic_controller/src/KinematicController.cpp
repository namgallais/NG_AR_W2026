#include "kinematic_controller/KinematicController.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
KinematicController::KinematicController()
: Node("kinematic_controller"),
  current_joint_state(Eigen::VectorXd::Zero(7)),
  current_joint_velocity(Eigen::VectorXd::Zero(7)),
  has_joint_state(false),
  x_fbk(Eigen::Vector3d::Zero()),
  rotation_fbk(Eigen::Matrix3d::Zero()),
  quat_fbk(Eigen::Quaterniond::Identity()),
  fbk_twist(Eigen::VectorXd::Zero(6)),
  end_eff_desired_velocity(Eigen::VectorXd::Zero(6)),
  desired_joint_velocity_vector(Eigen::VectorXd::Zero(7)),
  ref_velocity_vector(Eigen::VectorXd::Zero(7)),
  has_ref_vel(false)
{

  desired_joint_velocity_msg.data.resize(7);
  desired_joint_velocity_msg.data.assign(7, 0.0);


/*----------------PARAMETERS----------------*/

  if (!readParameters()) {
        RCLCPP_INFO(this->get_logger(), "Cannot read parameters");
        rclcpp::shutdown() ;
  } 
    else {
        RCLCPP_INFO(this->get_logger(), "Successfully read parameters");
  }

/*----------------Pinocchio------------*/

  pinocchio::urdf::buildModel(urdf_file_name, model, false);
  data = pinocchio::Data(model);

  HAND_ID = model.getJointId("bracelet_link") - 1 ;

  RCLCPP_INFO_STREAM(this->get_logger(),
    "Model nq: " << model.nq << " nv: " << model.nv);
  
  J.resize(6, model.nq); //need model.nv to initialize matricies.
  J_pinv.resize(model.nq, 6);
  Jv.resize(3, model.nq);
  Jv_pinv.resize(model.nq, 3);
  N.resize(model.nq, model.nq);
  I = Eigen::MatrixXd::Identity(model.nq, model.nq);

/*---------------PUBLISHER AND SUBSCRIBER---------------*/

// Forward Kinematic to know end-effector pose
  joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&KinematicController::jointCallback, this, std::placeholders::_1)
  );
  
  pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(
    "/gen3/feedback/pose", 10);      //Pose and Twist pub = foward kinematics from the current_joint_states

  twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    "/gen3/feedback/twist", 10);

// Inverse Kinematic to publish joint desired velocity
  ref_twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
  "/gen3/reference/twist", 
  10,
  std::bind(&KinematicController::twistCallback, this, std::placeholders::_1)
  );
  
  ref_velocity_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
  "/gen3/reference/velocity", 
  10,
  std::bind(&KinematicController::ref_vel_Callback, this, std::placeholders::_1)
  );
  
  desired_joint_velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
  "/gen3/command/joint_velocity", 10);

//500 Hz
  timer = this->create_wall_timer(
    2ms,
    std::bind(&KinematicController::update, this)
  );

  RCLCPP_INFO(this->get_logger(), "Potential field planner started");
}

/* ================= CALLBACKS ================= */

void KinematicController::jointCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state = Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size());
  current_joint_velocity = Eigen::Map<Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());
  has_joint_state = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First joint state callback");
}

void KinematicController::twistCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{

  end_eff_desired_velocity(0) = msg->linear.x;
  end_eff_desired_velocity(1) = msg->linear.y;
  end_eff_desired_velocity(2) = msg->linear.z;
  end_eff_desired_velocity(3) = msg->angular.x;
  end_eff_desired_velocity(4) = msg->angular.y;
  end_eff_desired_velocity(5) = msg->angular.z;


  RCLCPP_INFO_ONCE(this->get_logger(), "First reference twist callback");
}

void KinematicController::ref_vel_Callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (int i = 0; i < 7; i++) {
    ref_velocity_vector(i) = msg->data[i];
  }
  has_ref_vel = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First reference velocity callback");
}

void KinematicController::update( )
{
  
  //Forward Kinematics
  if (!has_joint_state) return;
  
  pinocchio::computeAllTerms(model, data, current_joint_state, current_joint_velocity) ; // compute all kinematics and dynamics related values
  pinocchio::getJointJacobian(model, data, HAND_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J) ; // get jacobian
	

  
  
  fbk_pose = data.oMi[HAND_ID];
  
  x_fbk = fbk_pose.translation(); //return an eigen vector
  rotation_fbk = fbk_pose.rotation();
  
  pinocchio::quaternion::assignQuaternion(quat_fbk, rotation_fbk);	
  
  pose_msg.position.x = x_fbk(0);
  pose_msg.position.y = x_fbk(1);
  pose_msg.position.z = x_fbk(2);
  pose_msg.orientation.w = quat_fbk.w();
  pose_msg.orientation.x = quat_fbk.x();
  pose_msg.orientation.y = quat_fbk.y();
  pose_msg.orientation.z = quat_fbk.z();

  fbk_twist = J * current_joint_velocity;
  
  twist_msg.linear.x = fbk_twist(0);
  twist_msg.linear.y = fbk_twist(1);
  twist_msg.linear.z = fbk_twist(2);

  twist_msg.angular.x = fbk_twist(3);
  twist_msg.angular.y = fbk_twist(4);
  twist_msg.angular.z = fbk_twist(5);

  pose_pub->publish(pose_msg);
  twist_pub->publish(twist_msg);
  
  
  //Inverse Kinematics
  
  if(control_orientation){
  
    J_pinv = J.transpose() * ((J * J.transpose()) + ((lambda * lambda)*I6)).inverse(); // (7*6) * ( (6*7)*(7*6) ) = (7*3)*(3*3) = (7*6)
  
    desired_joint_velocity_vector = J_pinv * end_eff_desired_velocity; // (7*6) * (6*1) = (7*1)
  
    if(with_redundancy){
      if (!has_ref_vel) return;
      N = I - (J_pinv * J); // (7*7) + (7*6)*(6*7) = (7*7)
    
      desired_joint_velocity_vector += (N*ref_velocity_vector);
    }
    
  }else{
    Jv = J.topRows(3); //  Jv   <- linear part (rows 0,1,2) (3*7)
  
    //Jv_pinv =  (Jv.transpose())*((Jv * Jv.transpose()).inverse()); // (7*3) * ( (3*7)*(7*3) ) = (7*3)*(3*3) = (7*3)
  
    /*======*/
    Jv_pinv = Jv.transpose() * ((Jv * Jv.transpose()) + ((lambda * lambda)*I3)).inverse();
    /*=====*/
  
    desired_joint_velocity_vector = Jv_pinv * end_eff_desired_velocity.topRows(3); // (7*3) * (3*1) = (7*1)
  
    if(with_redundancy){
      if (!has_ref_vel) return;
      N = I - (Jv_pinv * Jv); // (7*7) + (7*3)*(3*7) = (7*7)
    
      desired_joint_velocity_vector += (N*ref_velocity_vector);
    }
  }
  
  for(int i = 0; i < 7; i++) {
  desired_joint_velocity_msg.data[i] = desired_joint_velocity_vector(i);
  }
 desired_joint_velocity_pub->publish(desired_joint_velocity_msg);
}


bool KinematicController::readParameters()
{
  
  this->declare_parameter<std::string>("urdf_file_name", "");
  this->declare_parameter<bool>("with_redundancy", false);
  this->declare_parameter<bool>("control_orientation", false);

  
  /* -------- urdf_file_name -------- */  
  if (this->get_parameter("urdf_file_name", urdf_file_name)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "urdf_file_name = " << urdf_file_name);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get urdf_file_name");
    return false;
  }
  
   /* -------- with_redundancy -------- */  
  if (this->get_parameter("with_redundancy", with_redundancy)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "with_redundancy = " << with_redundancy);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get with_redundancy");
    return false;
  }
  
  /* -------- control_orientation -------- */  
  if (this->get_parameter("control_orientation", control_orientation)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "control_orientation = " << control_orientation);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get control_orientation");
    return false;
  }


  return true;
}


