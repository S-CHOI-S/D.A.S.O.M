/*******************************************************************************
* D.A.S.O.M
*
* Department of Aerial Manipulator System for Object Manipulation
*
*     https://github.com/S-CHOI-S/D.A.S.O.M.git
*
* Mobile Robotics Lab. (MRL)
* 	  @ Seoul National University of Science and Technology
*
*	  https://mrl.seoultech.ac.kr/index.do
*
*******************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#include "dasom_controllers/dasom_manipulator_control.h"

DasomControl::DasomControl()
: node_handle_(""), priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  ds_jnt1_ = new DasomJoint(4,4); // admittance cof 4?
  ds_jnt2_ = new DasomJoint(4,4);
  ds_jnt3_ = new DasomJoint(4,4);
  ds_jnt4_ = new DasomJoint(4,4);
  ds_jnt5_ = new DasomJoint(4,4);
  ds_jnt6_ = new DasomJoint(4,4);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();

  ROS_INFO("Dasom Manipulator Control node start");
  ROS_INFO("===========================================");
  ROS_INFO("[3.2s] Rise to init pose");
  ROS_INFO("[5.0s] DOB ON");

  /************************************************************
  ** Resize matrices & vectors
  ************************************************************/
  // For init pose
  initPoseFlag = true; // default = true
  haptic_offset.resize(6,1);
  initPose.resize(6,1);
  initPose_for_initPoseFunction.resize(6,1);
  angle_init.resize(6);
  haptic_offset << -0.0391781188965, 0, 0.0570636393229, 0, 0, 0;
  initPose << 0, 0.2, 0.3, M_PI/2, 0, 0;
  initPose_for_initPoseFunction = initPose + haptic_offset;
  angle_init = InverseKinematics(initPose_for_initPoseFunction);

  // For angle control
  angle_ref.resize(6);
  angle_d.resize(6);
  angle_measured.resize(6,1);
  angle_ref << 0, 0, 0, 0, 0, 0;

  // For angle safe
  angle_safe.resize(6,1);
  angle_max.resize(6,1);
  angle_min.resize(6,1);
  angle_safe << 0, 0, 0, 0, 0, 0;
  angle_max << 10, 10, 10, 10, 10, 10;
  angle_min << -10, -10, -10, -10, -10, -10;
  // angle_max << 1.57, 2.5, 0, 10, 10, 10;
  // angle_min << -1.57, 0, -1.8, -10, -10, -10;

  // For pose control
  FK_pose.resize(6,1);
  EE_command.resize(6);
  EE_command_vel_limit.resize(6);
  FK_pose = initPose;
  EE_command = initPose;
  EE_command_vel_limit = initPose_for_initPoseFunction;
  
  // For haptic control
  haptic_initPose.resize(6,1); // Not in use
  haptic_command.resize(6);
  haptic_command << 0, 0, 0, 0, 0, 0;
  
  // For haptic button
  grey_button = 0;
  white_button = true;
  gripper_cmd = 0;

  // For gimbaling
  gimbal_tf.resize(7);
  gimbal_EE_cmd.resize(6);
  global_EE_tf.resize(7);
  gimbal_EE_cmd = initPose;

  // For force estimation
  J.resize(6,6);
  JT.resize(6,6);
  JTI.resize(6,6);
  velocity_measured.resize(6,1);
  tau_measured.resize(6,1);
  F_ext.resize(6,1);
  F_max.resize(3,1);
  F_min.resize(3,1);
  hysteresis_max.resize(6,1);
  hysteresis_min.resize(6,1);
  F_ext << 0, 0, 0, 0, 0, 0;
  F_max << 2.0, 1.0, 0;
  F_min << -2.0, -1.0, 0;
  hysteresis_max << 0.5, 1, 5, 0, 0, 0;
  hysteresis_min << -0.5, -1, -5, 0, 0, 0;

  // For admittance control
  X_ref.resize(6,1);
  X_cmd.resize(6,1);
  X_ref = initPose;
  X_cmd = initPose;
  
  // For DOB
  d_hat.resize(6);
 
  ros::Rate init_rate(1);

  // For bpf
  wl = 1;
  wh = 2;
  w = sqrt(wl * wh);
  Q = w / (wh - wl);
  
  //pf//
  bp_A << - w/ Q, - w*w,
              1,      0;

  bp_B << 1, 0;
  bp_B.transpose();
  bp_C << w/ Q, 0;
  bp_D = 0;
  bp_X1 << 0, 0;
  bp_X1.transpose();
  bp_X2 << 0, 0;
  bp_X2.transpose();
  bp_X3 << 0, 0;
  bp_X3.transpose();
}

DasomControl::~DasomControl()
{
  deleteToolbox();

  ROS_INFO("Bye!");
  ros::shutdown();
}

void DasomControl::initPublisher()
{
  joint_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/goal_dynamixel_position", 10);
  joint_measured_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/measured_dynamixel_position", 10);  
  dasom_EE_pos_pub_ = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/EE_pose", 10); //use
  gimbal_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>(robot_name_ + "/tf/global_fixed_gimbal_EE_pose", 10); //use
  force_pub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>(robot_name_ + "/external_force", 10); //use

  // For Test
  test_Pub = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/test_Pub", 10); //use
  test_Pub2 = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/test_Pub2", 10); //use
}

void DasomControl::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe(robot_name_ + "/joint_states", 10, &DasomControl::jointCallback, this, ros::TransportHints().tcpNoDelay());
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy/dasom", 10, &DasomControl::joystickCallback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &DasomControl::buttonCallback, this, ros::TransportHints().tcpNoDelay());
  gimbal_cmd_sub_ = node_handle_.subscribe(robot_name_ + "/tf/global_gimbal_command", 10, &DasomControl::gimbalEECmdCallback, this, ros::TransportHints().tcpNoDelay()); //use
  global_EE_pose_sub_ = node_handle_.subscribe(robot_name_ + "/tf/global_EE_pose", 10, &DasomControl::globalEEPoseCallback, this, ros::TransportHints().tcpNoDelay());
}

void DasomControl::initServer()
{
  admittance_srv_ = node_handle_.advertiseService(robot_name_ + "/admittance_srv", &DasomControl::admittanceCallback, this);
  bandpass_srv_ = node_handle_.advertiseService(robot_name_ + "/bandpass_srv", &DasomControl::bandpassCallback, this);
}

void DasomControl::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);
  angle_measured[2] = msg->position.at(2);
  angle_measured[3] = msg->position.at(3);
  angle_measured[4] = msg->position.at(4);
  angle_measured[5] = msg->position.at(5);

  velocity_measured[0] = msg->velocity.at(0);
  velocity_measured[1] = msg->velocity.at(1);
  velocity_measured[2] = msg->velocity.at(2);
  velocity_measured[3] = msg->velocity.at(3);
  velocity_measured[4] = msg->velocity.at(4);
  velocity_measured[5] = msg->velocity.at(5);

  tau_measured[0] = msg->effort.at(0); 
  tau_measured[1] = msg->effort.at(1); 
  tau_measured[2] = msg->effort.at(2); 
  tau_measured[3] = msg->effort.at(3); 
  tau_measured[4] = msg->effort.at(4); 
  tau_measured[5] = msg->effort.at(5);
}

void DasomControl::joystickCallback(const geometry_msgs::Twist &msg)
{
  haptic_command[0] = msg.linear.x;
  haptic_command[1] = msg.linear.y;
  haptic_command[2] = msg.linear.z;
}

void DasomControl::buttonCallback(const omni_msgs::OmniButtonEvent &msg)
{
  if(msg.grey_button == 1) 
  {
    if(grey_button == 0)
    // For gimbaling
    {
      grey_button++;
      ROS_INFO("Grey 1: Gimbaling mode");

      gimbal_tf = global_EE_tf; // globalEEPoseCallback

      geometry_msgs::PoseStamped gimbal_tf_msg;

      gimbal_tf_msg.pose.position.x = gimbal_tf[0];
      gimbal_tf_msg.pose.position.y = gimbal_tf[1];
      gimbal_tf_msg.pose.position.z = gimbal_tf[2];

      gimbal_tf_msg.pose.orientation.x = gimbal_tf[3];
      gimbal_tf_msg.pose.orientation.y = gimbal_tf[4];
      gimbal_tf_msg.pose.orientation.z = gimbal_tf[5];
      gimbal_tf_msg.pose.orientation.w = gimbal_tf[6];

      gimbal_pub_.publish(gimbal_tf_msg);
    }
    else if(grey_button == 1)
    // For gimbaling + command mode
    {
      grey_button++;
      ROS_INFO("Grey 2: Gimbaling + Command mode");
    }
    else if(grey_button == 2)
    // For command mode
    {
      grey_button = 0;
      ROS_INFO("Grey 0: Command mode");

      gimbalcommand_safe = false;
    }
  }

  if(msg.white_button == 1) 
  {
    if(white_button)
    {
      white_button = false;
      ROS_INFO("white: false");
      gripper_cmd = 2.3;
    }
    else
    {
      white_button = true;
      ROS_INFO("white: true");
      gripper_cmd = 0;
    }
  }
}

void DasomControl::globalEEPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  global_EE_tf[0] = msg.pose.position.x;
  global_EE_tf[1] = msg.pose.position.y;
  global_EE_tf[2] = msg.pose.position.z;

  global_EE_tf[3] = msg.pose.orientation.x;
  global_EE_tf[4] = msg.pose.orientation.y;
  global_EE_tf[5] = msg.pose.orientation.z;
  global_EE_tf[6] = msg.pose.orientation.w;

  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);

  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  quat.setRPY(roll, pitch, yaw);

  // ROS_INFO("globalEEPoseCallback = %lf, %lf, %lf", roll, pitch, yaw);
}

void DasomControl::gimbalEECmdCallback(const geometry_msgs::PoseStamped &msg)
// 최종 gimbal command
{
  gimbal_EE_cmd[0] = msg.pose.position.x;
  gimbal_EE_cmd[1] = msg.pose.position.y;
  gimbal_EE_cmd[2] = msg.pose.position.z;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation,quat);

  tf::Matrix3x3(quat).getRPY(gimbal_EE_cmd[3], gimbal_EE_cmd[4], gimbal_EE_cmd[5]);

  // ROS_WARN("gimbalEECmdCallback = %lf, %lf, %lf", gimbal_EE_cmd[3], gimbal_EE_cmd[4], gimbal_EE_cmd[5]);
}

bool DasomControl::admittanceCallback(dasom_controllers::admittanceSRV::Request  &req,
                                      dasom_controllers::admittanceSRV::Response &res)
{
  virtual_mass_x = req.x_m;
  virtual_damper_x = req.x_d;
  virtual_spring_x = req.x_k;
  
  virtual_mass_y = req.y_m;
  virtual_damper_y = req.y_d;
  virtual_spring_y = req.y_k;

  virtual_mass_z = req.z_m;
  virtual_damper_z = req.z_d;
  virtual_spring_z = req.z_k;

  initializeAdmittance();

  X_from_model_matrix << 0, 0;
  X_dot_from_model_matrix << 0, 0;
  Y_from_model_matrix << 0, 0;
  Y_dot_from_model_matrix << 0, 0;
  Z_from_model_matrix << 0, 0;
  Z_dot_from_model_matrix << 0, 0;

  return true;
}

bool DasomControl::bandpassCallback(dasom_controllers::bandpassSRV::Request & req,
                                    dasom_controllers::bandpassSRV::Response & res)
{
  wl = req.wl;
  wh = req.wh;
  double w = sqrt(wl * wh);
  double Q = w / (wh - wl);

  ROS_INFO("wl = %lf", wl);
  ROS_INFO("wh = %lf", wh);
  
  //pf//
  bp_A << - w/ Q, - w*w,
               1,     0;

  bp_B << 1, 0;
  bp_B.transpose();
  bp_C << w/ Q, 0;
  bp_D = 0;
  bp_X1 << 0, 0;
  bp_X1.transpose();
  bp_X2 << 0, 0;
  bp_X2.transpose();
  bp_X3 << 0, 0;
  bp_X3.transpose();

  return true;
}

double DasomControl::tanh_function(double input_data, double cut_off_force)
{
  double data = input_data / cut_off_force * 4;
  return abs((exp(data) - exp(-data)) / (exp(data) + exp(-data)));
}

void DasomControl::tauLPFforExternalForce()
{
  tau_measured[0] = ds_jnt1_->updateLPF(time_loop, tau_measured[0]);
  tau_measured[1] = ds_jnt2_->updateLPF(time_loop, tau_measured[1]);
  tau_measured[2] = ds_jnt3_->updateLPF(time_loop, tau_measured[2]);
  tau_measured[3] = ds_jnt4_->updateLPF(time_loop, tau_measured[3]);
  tau_measured[4] = ds_jnt5_->updateLPF(time_loop, tau_measured[4]);
  tau_measured[5] = ds_jnt6_->updateLPF(time_loop, tau_measured[5]);
}

void DasomControl::CalcExternalForce()
{
  geometry_msgs::WrenchStamped ext_force;

  // tauLPFforExternalForce();

  J = Jacobian(angle_measured);
  JT = J.transpose();
  JTI = JT.inverse(); // completeOrthogonalDecomposition().pseudoInverse();

  KDLrun(angle_measured, velocity_measured); // Compute MCG Dynamics

  F_ext = JTI * (tau_measured - C_matrix - G_matrix);

  bf_F_ext[0] = F_ext[0];
  bf_F_ext[1] = F_ext[1];
  bf_F_ext[2] = F_ext[2];

  // band pass filter
  // bf_F_ext[0] = F_ext[0]; // raw data
  // bp_X_dot1 = bp_A * bp_X1 + bp_B * F_ext[0];
  // bp_X1 += bp_X_dot1 * time_loop;
  // bf_F_ext[0] = bp_C.dot(bp_X1) + F_ext[0] * bp_D;

  // bp_X_dot2 = bp_A * bp_X2 + bp_B * F_ext[1];
  // bp_X2 += bp_X_dot2 * time_loop;
  // bf_F_ext[1] = bp_C.dot(bp_X2) + F_ext[1] * bp_D;

  // bp_X_dot3 = bp_A * bp_X3 + bp_B * F_ext[2];
  // bp_X3 += bp_X_dot3 * time_loop;
  // bf_F_ext[2] = bp_C.dot(bp_X3) + F_ext[2] * bp_D;

  // if(bf_F_ext[2] < 0) bf_F_ext[2] = bf_F_ext[2] * 0.7;

  // Hyperbolic tangent
  bf_F_ext_tanh[0] = tanh_function(bf_F_ext[0], 10) * bf_F_ext[0];
  bf_F_ext_tanh[1] = tanh_function(bf_F_ext[1], 60) * bf_F_ext[1];
  bf_F_ext_tanh[2] = tanh_function(bf_F_ext[2], 30) * bf_F_ext[2];

  // if (bf_F_ext[0] <= hysteresis_max[0] && bf_F_ext[0] >= hysteresis_min[0]) bf_F_ext[0] = 0;
  // else if (bf_F_ext[0] > hysteresis_max[0]) bf_F_ext[0] -= hysteresis_max[0];
  // else if (bf_F_ext[0] < hysteresis_min[0]) bf_F_ext[0] -= hysteresis_min[0];

  // if (bf_F_ext[1] <= hysteresis_max[1] && bf_F_ext[1] >= hysteresis_min[1]) bf_F_ext[1] = 0;
  // else if (bf_F_ext[1] > hysteresis_max[1]) bf_F_ext[1] -= hysteresis_max[1];
  // else if (bf_F_ext[1] < hysteresis_min[1]) bf_F_ext[1] -= hysteresis_min[1];

  // if (bf_F_ext[2] <= hysteresis_max[2] && bf_F_ext[2] >= hysteresis_min[2]) bf_F_ext[2] = 0;
  // else if (bf_F_ext[2] > hysteresis_max[2]) bf_F_ext[2] -= hysteresis_max[2];
  // else if (bf_F_ext[2] < hysteresis_min[2]) bf_F_ext[2] -= hysteresis_min[2];

  // bf_F_ext_tanh: only hyperbolic tangent
  // bf_F_ext: dead zone!

  ext_force.header.stamp = ros::Time::now();

  ext_force.wrench.force.x = F_ext[0];
  ext_force.wrench.force.y = F_ext[1];
  ext_force.wrench.force.z = F_ext[2];
  ext_force.wrench.torque.x = bf_F_ext_tanh[0];
  ext_force.wrench.torque.y = bf_F_ext_tanh[1];
  ext_force.wrench.torque.z = bf_F_ext_tanh[2];

  force_pub_.publish(ext_force); // all 8 20 0 (8 20 20)
}

void DasomControl::ForceGenerator()
{
  ROS_INFO("cnt_force = %d", cnt_force);

  if(cnt_force > 1000 && cnt_force < 1100)
  {
    bf_F_ext_tanh[0] = 1;// 0.5* sin(cnt_force*2*M_PI/2000);
    bf_F_ext_tanh[1] = 1;
    bf_F_ext_tanh[2] = 1;
    
    ROS_INFO("bf_F_ext_tanh[0] = %lf", bf_F_ext_tanh[0]);
  }

  cnt_force++;
}

void DasomControl::AdmittanceControl()
// X_ref: haptic command
{
  // 2 10 6(bp 1 3) // 10 15 10(bp 2 8)
  X_cmd[0] = admittanceControlX(time_loop, X_ref[0], bf_F_ext_tanh[0]);

  // 0.1 3 1(bp 1 3) // 5 20 10(bp 2 8)
  X_cmd[1] = admittanceControlY(time_loop, X_ref[1], bf_F_ext_tanh[1]);

  // 2 5 4(bp 1 3) // 5 20 10(bp 2 8)
  X_cmd[2] = admittanceControlZ(time_loop, X_ref[2], bf_F_ext_tanh[2]);

  EE_command[0] = X_cmd[0];
  EE_command[1] = X_cmd[1];
  EE_command[2] = X_cmd[2];
  EE_command[3] = X_ref[3]; // PI/2
  EE_command[4] = X_ref[4]; // 0
  EE_command[5] = X_ref[5];
}

void DasomControl::DOB()
{
  dob_cnt++;

  if(dob_cnt > 600)
  {
    // ROS_WARN("DOB Start!");
    d_hat[1] = ds_jnt2_->updateDOB(time_loop, angle_measured[1], angle_d[1]);
    if(d_hat[1] > 0.3) d_hat[1] = 0.3;
    else if (d_hat[1] < -0.3) d_hat[1] = -0.3;
    angle_d[1] = angle_safe[1] - d_hat[1];
    angle_safe[1] = angle_d[1];

    d_hat[2] = ds_jnt3_->updateDOB(time_loop, angle_measured[2], angle_d[2]);
    if(d_hat[2] > 0.3) d_hat[2] = 0.3;
    else if (d_hat[2] < -0.3) d_hat[2] = -0.3;
    angle_d[2] = angle_safe[2] - d_hat[2];
    angle_safe[2] = angle_d[2];

    d_hat[3] = ds_jnt4_->updateDOB(time_loop, angle_measured[3], angle_d[3]);
    angle_d[3] = angle_ref[3] - d_hat[3];
    angle_safe[3] = angle_d[3];

    d_hat[4] = ds_jnt5_->updateDOB(time_loop, angle_measured[4], angle_d[4]);
    angle_d[4] = angle_ref[4] - d_hat[4];
    angle_safe[4] = angle_d[4];

    d_hat[5] = ds_jnt6_->updateDOB(time_loop, angle_measured[5], angle_d[5]);
    angle_d[5] = angle_ref[5] - d_hat[5];
    angle_safe[5] = angle_d[5];
  }
}

void DasomControl::AngleSafeFunction()
{
  if (angle_ref[0] > angle_max[0] || angle_ref[0] < angle_min[0]) 
  {
    ROS_ERROR("angle 1 LIMIT");
  }
  if (angle_ref[1] > angle_max[1] || angle_ref[1] < angle_min[1]) 
  {
    ROS_ERROR("angle 2 LIMIT");
  }
  if (angle_ref[2] > angle_max[2] || angle_ref[2] < angle_min[2]) 
  {
    ROS_ERROR("angle 3 LIMIT");
  }
  if (angle_ref[3] > angle_max[3] || angle_ref[3] < angle_min[3]) 
  {
    ROS_ERROR("angle 4 LIMIT");
  }
  if (angle_ref[4] > angle_max[4] || angle_ref[4] < angle_min[4]) 
  {
    ROS_ERROR("angle 5 LIMIT");
  }
  if (angle_ref[5] > angle_max[5] || angle_ref[5] < angle_min[5]) 
  {
    ROS_ERROR("angle 6 LIMIT");
  }

  if (std::isnan(angle_ref[0]) || std::isnan(angle_ref[1]) || std::isnan(angle_ref[2]) ||
      std::isnan(angle_ref[3]) || std::isnan(angle_ref[4]) || std::isnan(angle_ref[5]))
  {
    ROS_WARN("Out of Workspace");
  }

  // For constraint
  if ((angle_ref[0] > angle_max[0] || angle_ref[0] < angle_min[0]) ||
      (angle_ref[1] > angle_max[1] || angle_ref[1] < angle_min[1]) ||
      (angle_ref[2] > angle_max[2] || angle_ref[2] < angle_min[2]) ||
      (angle_ref[3] > angle_max[3] || angle_ref[3] < angle_min[3]) ||
      (angle_ref[4] > angle_max[4] || angle_ref[4] < angle_min[4]) ||
      (angle_ref[5] > angle_max[5] || angle_ref[5] < angle_min[5]) ||
      std::isnan(angle_ref[0]) || std::isnan(angle_ref[1]) || std::isnan(angle_ref[2]) ||
      std::isnan(angle_ref[3]) || std::isnan(angle_ref[4]) || std::isnan(angle_ref[5])
     )
  {
    ROS_INFO("IK error");
  }
  else
  {
    angle_safe[0] = angle_ref[0];
    angle_safe[1] = angle_ref[1];
    angle_safe[2] = angle_ref[2];
    angle_safe[3] = angle_ref[3];
    angle_safe[4] = angle_ref[4];
    angle_safe[5] = angle_ref[5];
  }
}

void DasomControl::startGimbalHapticCommand()
{
  if(abs(haptic_command[0]) < 0.02 &&
     abs(haptic_command[1]) < 0.02 &&
     abs(haptic_command[2]) < 0.02)
  {
    cnt_gimbalcommand++;

    if(cnt_gimbalcommand >= 20) gimbalcommand_safe = true;
  }
  else
  {
    cnt_gimbalcommand = 0;
    gimbalcommand_safe = false;
  }
}

void DasomControl::CommandGenerator()
// EE_command: [haptic command](X) [IK command](O)
{
  if(grey_button == 0) 
  // For command mode
  {
    EE_command = haptic_command + initPose;
  }
  else if(grey_button == 1)
  // For gimbaling mode
  {
    EE_command = gimbal_EE_cmd;
  }
  else if(grey_button == 2)
  // For gimbaling + command mode
  {
    if(!gimbalcommand_safe)
    {
      startGimbalHapticCommand();
      EE_command = gimbal_EE_cmd;
    }
    else
    {
      EE_command = gimbal_EE_cmd + haptic_command;
    }
  }

  // For admittance
  X_ref = EE_command;
}

void DasomControl::CommandVelocityLimit()
{
  double vel_limit = 0.1;  // [m/s]

  for (int i = 0; i < 3; i++)
  {
    if((EE_command[i] - EE_command_vel_limit[i]) > 0.001) 
    {
      EE_command_vel_limit[i] = EE_command_vel_limit[i] + vel_limit * time_loop;
    }
    else if ((EE_command[i] - EE_command_vel_limit[i]) < - 0.001) 
    {
      EE_command_vel_limit[i] = EE_command_vel_limit[i] - vel_limit * time_loop;     
    }
    else
    {
      EE_command_vel_limit[i] = EE_command[i];
    }
  }

  EE_command_vel_limit[3] = EE_command[3];
  EE_command_vel_limit[4] = EE_command[4];
  EE_command_vel_limit[5] = EE_command[5];
}

void DasomControl::SolveInverseKinematics()
{
  geometry_msgs::Twist msg;

  InverseKinematics(EE_command_vel_limit);

  angle_ref = InverseKinematics(EE_command_vel_limit);

  FK_pose = EE_pose(angle_measured);

  msg.linear.x = FK_pose[0];
  msg.linear.y = FK_pose[1];
  msg.linear.z = FK_pose[2];
  msg.angular.x = FK_pose[3];
  msg.angular.y = FK_pose[4];
  msg.angular.z = FK_pose[5];

  dasom_EE_pos_pub_.publish(msg);
}

void DasomControl::initPoseFunction()
{
  for (int i = 0; i < 6; i++)
  {
    if((angle_init[i] - angle_ref[i]) > 0.001) 
    {
      angle_ref[i] = angle_ref[i] + 0.0005;
    }
    else if ((angle_init[i] - angle_ref[i]) < - 0.001) 
    {
      angle_ref[i] = angle_ref[i] - 0.0005; // 0.01 * time_loop              
    }
    else
    {
      angle_ref[i] = angle_init[i];
    }
  }

  if ((abs(angle_init[0] - angle_ref[0]) < 0.001) &&
      (abs(angle_init[1] - angle_ref[1]) < 0.001) &&
      (abs(angle_init[2] - angle_ref[2]) < 0.001) &&
      (abs(angle_init[3] - angle_ref[3]) < 0.001) &&
      (abs(angle_init[4] - angle_ref[4]) < 0.001) &&
      (abs(angle_init[5] - angle_ref[5]) < 0.001)      
     )
  {
    initPoseFlag = false;
    ROS_WARN("Finished to arrive at the initial pose!");
    ROS_INFO("Grey 0: Command mode");
    
    ds_jnt1_->initDOB();
    ds_jnt2_->initDOB();
    ds_jnt3_->initDOB();
    ds_jnt4_->initDOB();
    ds_jnt5_->initDOB();
    ds_jnt6_->initDOB();
  }
}

void DasomControl::PublishData()
{
  sensor_msgs::JointState joint_cmd;
  sensor_msgs::JointState joint_measured;

  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.position.push_back(angle_safe[0]); 
  joint_cmd.position.push_back(angle_safe[1]); 
  joint_cmd.position.push_back(angle_safe[2]); 
  joint_cmd.position.push_back(angle_safe[3]); 
  joint_cmd.position.push_back(angle_safe[4]); 
  joint_cmd.position.push_back(angle_safe[5]);
  joint_cmd.position.push_back(gripper_cmd);

  // joint_cmd.velocity.push_back(tau_measured[0]);
  // joint_cmd.velocity.push_back(ds_jnt1_->updateLPF(time_loop, tau_measured[0]));
  // joint_cmd.velocity.push_back(G_matrix[0]);

  // joint_cmd.velocity.push_back(tau_measured[1]);
  // joint_cmd.velocity.push_back(ds_jnt2_->updateLPF(time_loop, tau_measured[1]));
  // joint_cmd.velocity.push_back(ds_jnt3_->updateLPF(time_loop, tau_measured[1]));
  // joint_cmd.velocity.push_back(G_matrix[1]);

  // joint_cmd.velocity.push_back(tau_measured[5]);
  // joint_cmd.velocity.push_back(ds_jnt2_->updateLPF(time_loop, tau_measured[5]));
  // joint_cmd.velocity.push_back(ds_jnt3_->updateLPF(time_loop, tau_measured[5]));
  // joint_cmd.velocity.push_back(G_matrix[5]);

  // joint_cmd.velocity.push_back(tau_measured[3]);
  // joint_cmd.velocity.push_back(ds_jnt4_->updateLPF(time_loop, tau_measured[3]));
  // joint_cmd.velocity.push_back(G_matrix[3]);

  // joint_cmd.velocity.push_back(tau_measured[4]);
  // joint_cmd.velocity.push_back(ds_jnt5_->updateLPF(time_loop, tau_measured[4]));
  // joint_cmd.velocity.push_back(G_matrix[4]);

  // joint_cmd.velocity.push_back(tau_measured[5]);
  // joint_cmd.velocity.push_back(ds_jnt6_->updateLPF(time_loop, tau_measured[5]));
  // joint_cmd.velocity.push_back(G_matrix[5]);

  // joint_cmd.effort.push_back(angle_d[0]); 
  // joint_cmd.effort.push_back(angle_d[1]); 
  // joint_cmd.effort.push_back(angle_d[2]); 
  // joint_cmd.effort.push_back(angle_measured[4]); 
  // joint_cmd.effort.push_back(angle_measured[5]);  

  joint_command_pub_.publish(joint_cmd);

  // ROS_ERROR("==============================================================");
  // ROS_INFO("angle_measured = %lf, %lf, %lf, %lf, %lf, %lf", angle_measured[0], angle_measured[1],angle_measured[2],angle_measured[3],angle_measured[4],angle_measured[5]);
  // ROS_WARN("================================================");
  // ROS_INFO("angle_ref = %lf, %lf, %lf, %lf, %lf, %lf", angle_ref[0], angle_ref[1],angle_ref[2],angle_ref[3],angle_ref[4],angle_ref[5]);
  // ROS_WARN("================================================");
  // // ROS_INFO("angle_safe = %lf, %lf, %lf, %lf, %lf, %lf", angle_safe[0], angle_safe[1],angle_safe[2],angle_safe[3],angle_safe[4],angle_safe[5]);
  // // ROS_WARN("================================================");

  // ROS_INFO("angle_error = %lf, %lf, %lf, %lf, %lf, %lf", angle_ref[0]-angle_measured[0], angle_ref[1]-angle_measured[1],angle_ref[2]-angle_measured[2],angle_ref[3]-angle_measured[3],angle_ref[4]-angle_measured[4],angle_ref[5]-angle_measured[5]);
  // ROS_WARN("================================================");
}

void DasomControl::test()
{
  geometry_msgs::Twist first_publisher;
  geometry_msgs::Twist second_publisher;

  first_publisher.linear.x = EE_command_vel_limit[0];
  first_publisher.linear.y = EE_command_vel_limit[1];
  first_publisher.linear.z = EE_command_vel_limit[2];
  first_publisher.angular.x = EE_command_vel_limit[3];
  first_publisher.angular.y = EE_command_vel_limit[4];
  first_publisher.angular.z = EE_command_vel_limit[5];

  second_publisher.linear.x = X_ref[0];
  second_publisher.linear.y = X_ref[1];
  second_publisher.linear.z = X_ref[2];
  second_publisher.angular.x = X_ref[3];
  second_publisher.angular.y = X_ref[4];
  second_publisher.angular.z = X_ref[5];

  test_Pub.publish(first_publisher);
  test_Pub2.publish(second_publisher);
}

void DasomControl::deleteToolbox()
{
  delete ds_jnt1_;
  delete ds_jnt2_;
  delete ds_jnt3_;
  delete ds_jnt4_;
  delete ds_jnt5_;
  delete ds_jnt6_;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "DasomControl");
  DasomControl ds_ctrl_;

  ros::Rate init_rate(1);
  ros::spinOnce();
  init_rate.sleep();
  ros::spinOnce();
  ds_ctrl_.angle_ref = ds_ctrl_.angle_measured;
  ROS_WARN("Start moving to initPose!");

  ros::Rate loop_rate(200);
  ds_ctrl_.time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
    ds_ctrl_.time_f = ros::Time::now().toSec();
    ds_ctrl_.time_loop = ds_ctrl_.time_f - ds_ctrl_.time_i;
    ds_ctrl_.time_i = ros::Time::now().toSec();

    if(ds_ctrl_.initPoseFlag)
    {
      ds_ctrl_.initPoseFunction();
      ds_ctrl_.AngleSafeFunction();
    }
    else
    {
      ds_ctrl_.CommandGenerator();
      ds_ctrl_.CalcExternalForce();
      // ds_ctrl_.ForceGenerator();
      ds_ctrl_.AdmittanceControl();
      ds_ctrl_.CommandVelocityLimit();
      ds_ctrl_.SolveInverseKinematics();
      ds_ctrl_.AngleSafeFunction();
      ds_ctrl_.DOB();
    }
  
    ds_ctrl_.PublishData();
    ds_ctrl_.test();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
