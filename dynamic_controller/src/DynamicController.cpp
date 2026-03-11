#include "dynamic_controller/DynamicController.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/* ========== Constructor ========== */
DynamicController::DynamicController()
: Node("dynamic_controller"),
  fbk_q_position(Eigen::VectorXd::Zero(7)),
  fbk_q_velocity(Eigen::VectorXd::Zero(7)),

  fbk_end_eff_pose(Eigen::Vector3d::Zero()),
  fbk_twist(Eigen::VectorXd::Zero(6)),
  
  x_fbk(Eigen::Vector3d::Zero()),
  dx_fbk(Eigen::Vector3d::Zero()),
  
  ref_q_position(Eigen::VectorXd::Zero(7)),
  ref_q_velocity(Eigen::VectorXd::Zero(7)),

  x_ref(Eigen::Vector3d::Zero()),
  dx_ref(Eigen::Vector3d::Zero()),
  
  Lambda(Eigen::Matrix3d::Zero()),
  eta(Eigen::Vector3d::Zero()),
  
  ddx_cmd(Eigen::Vector3d::Zero()),

  tau_cmd(Eigen::VectorXd::Zero(7)),

  has_joint_state(false),
  has_ref_position(false),
  has_ref_velocity(false),
  has_ref_pose(false),
  has_ref_twist(false)
{

  desired_joint_effort_msg.data.resize(7);
  desired_joint_effort_msg.data.assign(7, 0.0);


/*----------------PARAMETERS----------------*/

  if (!readParameters()) {
        RCLCPP_INFO(this->get_logger(), "Cannot read parameters");
        rclcpp::shutdown() ;
  } 
    else {
        RCLCPP_INFO(this->get_logger(), "Successfully read parameters");
  }

/*----------------Pinocchio------------*/

  // build a pinocchio model
  pinocchio::urdf::buildModel(urdf_file_name, model, false) ;

    // data structure for pinocchio
  data  		  = pinocchio::Data(model) ;
  hand_id 	  = model.getJointId("bracelet_link") - 1 ; // bracelet_link, end-effector_link, and tool_frame will give the same ID
  dim_joints   = model.nq ;

  J	  = Eigen::MatrixXd::Zero(6, dim_joints) ;
  J_dot = Eigen::MatrixXd::Zero(6, dim_joints) ;
  M  		  = Eigen::MatrixXd::Identity(dim_joints,dim_joints) ;
  h 			  = Eigen::VectorXd::Zero(dim_joints) ;


  RCLCPP_INFO_STREAM(this->get_logger(),
    "Model nq: " << model.nq << " nv: " << model.nv);
  
  
  Jv.resize(3, model.nq);
  Jv_dot.resize(3, model.nq);
  Jv_pinv.resize(3, model.nq);
  P.resize(model.nq, model.nq);
  I = Eigen::MatrixXd::Identity(model.nq, model.nq);

/*---------------PUBLISHER AND SUBSCRIBER---------------*/

// Subscribers
  joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states",
  10,
  std::bind(&DynamicController::jointCallback, this, std::placeholders::_1)
  );

  ref_position_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
  "/gen3/reference/position",
  10,
  std::bind(&DynamicController::ref_position_Callback, this, std::placeholders::_1)
  );

  ref_velocity_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
  "/gen3/reference/velocity",
  10,
  std::bind(&DynamicController::ref_velocity_Callback, this, std::placeholders::_1)
  );
  
  ref_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
  "/gen3/reference/pose",
  10,
  std::bind(&DynamicController::ref_pose_Callback, this, std::placeholders::_1)
  );

  ref_twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
  "/gen3/reference/twist",
  10,
  std::bind(&DynamicController::ref_twist_Callback, this, std::placeholders::_1)
  );

// Publishers
  fbk_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(
  "/gen3/feedback/pose", 10);

  fbk_twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(
  "/gen3/feedback/twist", 10);

  desired_joint_effort_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
  "/joint_effort_controller/commands", 10);
  
//Services

  set_linear_kd_service =
    this->create_service<highlevel_interfaces::srv::SetKD>(
      "/gen3/set_linear_kd",
      std::bind(
        &DynamicController::set_linear_kd_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
    
  set_joint_kd_service =
    this->create_service<highlevel_interfaces::srv::SetKD>(
      "/gen3/set_joint_kd",
      std::bind(
        &DynamicController::set_joint_kd_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );


//500 Hz
  timer = this->create_wall_timer(
    2ms,
    std::bind(&DynamicController::update, this)
  );

  RCLCPP_INFO(this->get_logger(), "Potential field planner started");
}

/* ================= CALLBACKS ================= */

void DynamicController::jointCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  fbk_q_position = Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size());
  fbk_q_velocity = Eigen::Map<Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());
  has_joint_state = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First joint state callback");
}

void DynamicController::ref_position_Callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{

  for (int i = 0; i < 7; i++) {
    ref_q_position(i) = msg->data[i];
  }
  has_ref_position = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First joint state callback");
}

void DynamicController::ref_velocity_Callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (int i = 0; i < 7; i++) {
    ref_q_velocity(i) = msg->data[i];
  }
  has_ref_velocity = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First reference velocity callback");
}


void DynamicController::ref_pose_Callback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  
  x_ref(0) = msg->position.x;
  x_ref(1) = msg->position.y;
  x_ref(2) = msg->position.z;
  
  has_ref_pose = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First joint state callback");
}

void DynamicController::ref_twist_Callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{

  dx_ref(0) = msg->linear.x;
  dx_ref(1) = msg->linear.y;
  dx_ref(2) = msg->linear.z;

  has_ref_twist = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "First reference twist callback");
}


void DynamicController::set_linear_kd_callback(
  const std::shared_ptr<highlevel_interfaces::srv::SetKD::Request> request,
  std::shared_ptr<highlevel_interfaces::srv::SetKD::Response> response)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "First set_linear_kd_service callback");

  linear_k = request->k;
  linear_d = request->d;

  response->success = true;
}

void DynamicController::set_joint_kd_callback(
  const std::shared_ptr<highlevel_interfaces::srv::SetKD::Request> request,
  std::shared_ptr<highlevel_interfaces::srv::SetKD::Response> response)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "First set_joint_kd_service callback");

  joint_k = request->k;
  joint_d = request->d;

  response->success = true;
}

void DynamicController::update( )
{

  if (!has_joint_state) return;

  pinocchio::computeAllTerms(model, data, fbk_q_position, fbk_q_velocity) ; // compute all kinematics and dynamics related values
  pinocchio::getJointJacobian(model, data, hand_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J) ; // get J
  pinocchio::getJointJacobianTimeVariation(model, data, hand_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_dot) ; // time derivative of J

  fbk_end_eff_pose  = data.oMi[hand_id].translation() ; // end-effector translation
  M 				= data.M ; 	// mass matrix
  h 				= data.nle ;	// coriolis, centrifugal, and gravity
   
  RCLCPP_INFO_STREAM(this ->get_logger(), "fbk_q_position " << fbk_q_position.transpose() ) ;
  RCLCPP_INFO_STREAM(this ->get_logger(), "fbk_q_velocity " << fbk_q_velocity.transpose() ) ;
  RCLCPP_INFO_STREAM(this ->get_logger(), "M " << M ) ;
  RCLCPP_INFO_STREAM(this ->get_logger(), "h " << h.transpose() ) ;
  RCLCPP_INFO_STREAM(this ->get_logger(), "J " << J ) ;
  RCLCPP_INFO_STREAM(this ->get_logger(), "J_dot " << J_dot ) ;
  
  // Lambda and eta computation
  Jv = J.topRows(3);
  Jv_dot = J_dot.topRows(3);
  
  Lambda = (Jv * M.inverse() * Jv.transpose()).inverse();
  Jv_pinv = (Jv.transpose())*((Jv * Jv.transpose()).inverse());
  eta = (Jv_pinv.transpose()) * h - Lambda * Jv_dot * ref_q_velocity;


  //Foward Kinematic to figure out x_fbk and dx_fbk
  
  x_fbk = fbk_end_eff_pose;
  dx_fbk = Jv * fbk_q_velocity;
  
  // Calculation of Ẍcmd :
  
  ddx_cmd = linear_d * ( dx_ref - dx_fbk ) + linear_k * (x_ref -x_fbk );
  
  // Calculation of tau_cmd
  
  tau_cmd = (Jv.transpose()) * (Lambda * ddx_cmd + eta);
  /*
  if(with_redundancy){
    if (!has_ref_velocity) return;
    P = I - Jv.transpose()*Lambda*Jv*M.inverse();
    
    tau_cmd += (P*ref_q_velocity);
  }
  */
  // Publishing
  pose_msg.position.x = x_fbk(0);
  pose_msg.position.y = x_fbk(1);
  pose_msg.position.z = x_fbk(2);

  fbk_twist = J * fbk_q_velocity; //We compute and publish fbk_twist because dx_fbk is only 3*3 (here we also want angular velocities)
  
  twist_msg.linear.x = fbk_twist(0);
  twist_msg.linear.y = fbk_twist(1);
  twist_msg.linear.z = fbk_twist(2);

  twist_msg.angular.x = fbk_twist(3);
  twist_msg.angular.y = fbk_twist(4);
  twist_msg.angular.z = fbk_twist(5);

  fbk_pose_pub->publish(pose_msg);
  fbk_twist_pub->publish(twist_msg);
  

  for(int i = 0; i < 7; i++) {
  desired_joint_effort_msg.data[i] = tau_cmd(i);
  }
 desired_joint_effort_pub->publish(desired_joint_effort_msg);
}


bool DynamicController::readParameters()
{
  
  this->declare_parameter<std::string>("urdf_file_name", "");
  this->declare_parameter<bool>("with_redundancy", false);
  this->declare_parameter<double>("linear_k", 0.1);
  this->declare_parameter<double>("linear_d", 0.1);
  this->declare_parameter<double>("joint_k", 0.1);
  this->declare_parameter<double>("joint_d", 0.1);

  
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
  
  /* -------- linear_k -------- */  
  if (this->get_parameter("linear_k", linear_k)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "linear_k = " << linear_k);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get linear_k");
    return false;
  }

  /* -------- linear_d -------- */  
  if (this->get_parameter("linear_d", linear_d)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "linear_d = " << linear_d);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get linear_d");
    return false;
  }
  
  /* -------- joint_k -------- */  
  if (this->get_parameter("joint_k", joint_k)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "joint_k = " << joint_k);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get joint_k");
    return false;
  }
  
  /* -------- joint_d -------- */  
  if (this->get_parameter("joint_d", joint_d)) {
    RCLCPP_INFO_STREAM(this->get_logger(),
      "joint_d = " << joint_d);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Could not get joint_d");
    return false;
  }

  return true;
}


