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

  ds_jnt1_ = new DasomJoint(8,8);
  ds_jnt2_ = new DasomJoint(8,8);
  ds_jnt3_ = new DasomJoint(8,8);
  ds_jnt4_ = new DasomJoint(8,8);
  ds_jnt5_ = new DasomJoint(8,8);
  ds_jnt6_ = new DasomJoint(8,8);

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
  initPoseFlag = true;
  initPose.resize(6,1);
  angle_init.resize(6);
  initPose << 0, 0.23, 0.30, M_PI/2, 0, 0;
  angle_init = InverseKinematics(initPose);

  // For angle control
  angle_ref.resize(6);
  angle_measured.resize(6,1);
  angle_ref << 0, 0, 0, 0, 0, 0;

  // For angle safe
  angle_safe.resize(6,1);
  angle_max.resize(6,1);
  angle_min.resize(6,1);
  angle_safe << 0, 0, 0, 0, 0, 0;
  angle_max << 10, 100, 10, 10, 10, 10;
  angle_min << -10, -100, -10, -10, -10, -10;

  // For pose control
  FK_pose.resize(6,1);
  EE_command.resize(6);
  EE_command_vel_limit.resize(6);
  FK_pose << 0, 0.23, 0.30, M_PI/2, 0, 0;
  EE_command << 0, 0.23, 0.30, M_PI/2, 0, 0;
  EE_command_vel_limit << 0, 0.23, 0.30, M_PI/2, 0, 0;
  
  // For haptic control
  haptic_initPose.resize(6,1);
  haptic_command.resize(6);
  
  // For haptic button
  grey_button = true;
  white_button = true;
  gripper_cmd = 0;

  // For gimbaling
  gimbal_tf.resize(7);
  gimbal_EE_cmd.resize(6);
  global_EE_tf.resize(7);

  // For force estimation
  J.resize(6,6);
  JT.resize(6,6);
  JTI.resize(6,6);
  tau_ext.resize(6,1);
  tau_measured.resize(6,1);
  tau_gravity.resize(6,1);
  F_ext.resize(3,1);
  F_max.resize(3,1);
  F_min.resize(3,1);
  hysteresis_max.resize(6,1);
  hysteresis_min.resize(6,1);
  F_ext << 0, 0, 0;
  F_max << 2.0, 1.0, 0;
  F_min << -2.0, -1.0, 0;
  hysteresis_max << 0.35, 0.27, 0.2, 0, 0, 0;
  hysteresis_min << -0.24, -0.35, -0.2, 0, 0, 0;

  // For admittance control
  X_ref.resize(6,1);
  X_cmd.resize(6,1);
  
  // For DOB
  d_hat.resize(2);
 
  ros::Rate init_rate(1);
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
  gimbal_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>(robot_name_ + "/global_gimbal_pose", 10); //use

  // For Test
  test_Pub = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/test_Pub", 10); //use
  test_Pub2 = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/test_Pub2", 10); //use
}

void DasomControl::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &DasomControl::jointCallback, this, ros::TransportHints().tcpNoDelay());
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 10, &DasomControl::joystickCallback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &DasomControl::buttonCallback, this, ros::TransportHints().tcpNoDelay());
  gimbal_cmd_sub_ = node_handle_.subscribe(robot_name_ + "/gimbal_EE_cmd", 10, &DasomControl::gimbalCmdCallback, this, ros::TransportHints().tcpNoDelay()); //use
}

void DasomControl::initServer()
{

}

void DasomControl::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);
  angle_measured[2] = msg->position.at(2);
  angle_measured[3] = msg->position.at(3);
  angle_measured[4] = msg->position.at(4);
  angle_measured[5] = msg->position.at(5);

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
    if(grey_button)
    {
      grey_button = false;
      ROS_INFO("Grey: false");

      gimbal_tf = global_EE_tf;

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
    else
    {
      grey_button = true;
      ROS_INFO("Grey: true");
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

void DasomControl::gimbalCmdCallback(const geometry_msgs::PoseStamped &msg)
{
  gimbal_EE_cmd[0] = msg.pose.position.x;
  gimbal_EE_cmd[1] = msg.pose.position.y;
  gimbal_EE_cmd[2] = msg.pose.position.z;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation,quat);

  tf::Matrix3x3(quat).getRPY(gimbal_EE_cmd[3], gimbal_EE_cmd[4], gimbal_EE_cmd[5]);
}

void DasomControl::CalcExternalForce()
{
  J = Jacobian(angle_measured);
  JT = J.transpose();
  JTI = JT.inverse(); // completeOrthogonalDecomposition().pseudoInverse();

  tau_ext = tau_measured - tau_gravity;
}

void DasomControl::AdmittanceControl()
{
  X_cmd[0] = admittanceControlX(time_loop, X_ref[0], F_ext[0]);

  X_cmd[1] = admittanceControlY(time_loop, X_ref[1], F_ext[1]);

  X_cmd[2] = admittanceControlZ(time_loop, X_ref[2], F_ext[2]);
}

void DasomControl::DOB()
{
  d_hat[0] = ds_jnt1_->updateDOB(time_loop, angle_ref[0], angle_measured[0]);
  angle_ref[0] = angle_ref[0] - d_hat[0];

  d_hat[1] = ds_jnt2_->updateDOB(time_loop, angle_ref[1], angle_measured[1]);
  angle_ref[1] = angle_ref[1] - d_hat[1];

  d_hat[2] = ds_jnt3_->updateDOB(time_loop, angle_ref[2], angle_measured[2]);
  angle_ref[2] = angle_ref[2] - d_hat[2];

  d_hat[3] = ds_jnt4_->updateDOB(time_loop, angle_ref[3], angle_measured[3]);
  angle_ref[3] = angle_ref[3] - d_hat[3];

  d_hat[4] = ds_jnt5_->updateDOB(time_loop, angle_ref[4], angle_measured[4]);
  angle_ref[4] = angle_ref[4] - d_hat[4];

  d_hat[5] = ds_jnt6_->updateDOB(time_loop, angle_ref[5], angle_measured[5]);
  angle_ref[5] = angle_ref[5] - d_hat[5];
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

void DasomControl::CommandGenerator()
// EE_command: [haptic command](X) [IK command](O)
{
  if(grey_button == true) 
  {
    EE_command = haptic_command + initPose; 
    ROS_INFO("Command Mode");
  }
  else 
  {
    EE_command = gimbal_EE_cmd;
    ROS_INFO("Gimbaling Mode");
  }
}

void DasomControl::CommandVelocityLimit()
{
  for (int i = 0; i < 3; i++)
  {
    if((EE_command[i] - EE_command_vel_limit[i]) > 0.001) 
    {
      EE_command_vel_limit[i] = EE_command_vel_limit[i] + 0.0005;
    }
    else if ((EE_command[i] - EE_command_vel_limit[i]) < - 0.001) 
    {
      EE_command_vel_limit[i] = EE_command_vel_limit[i] - 0.0005; // 0.01 * time_loop              
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
      angle_ref[i] = angle_ref[i] - 0.0005;   //0.01 * time_loop              
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
    
    ds_jnt1_->initDOB();
    ds_jnt2_->initDOB();
    ds_jnt3_->initDOB();
    ds_jnt4_->initDOB();
    ds_jnt5_->initDOB();
    ds_jnt6_->initDOB();
  }
  // ROS_INFO("DOING");
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
  // joint_cmd.position.push_back(gripper_cmd);

  // joint_cmd.velocity.push_back(angle_d[1]); 
  // joint_cmd.velocity.push_back(angle_ref[1]); 
  // joint_cmd.velocity.push_back(angle_ref[2]); 
  // joint_cmd.velocity.push_back(angle_ref[3]); 
  // joint_cmd.velocity.push_back(angle_ref[4]); 
  // joint_cmd.velocity.push_back(angle_ref[5]); 

  // joint_cmd.effort.push_back(angle_measured[0]); 
  // joint_cmd.effort.push_back(angle_measured[1]); 
  // joint_cmd.effort.push_back(angle_measured[2]); 
  // joint_cmd.effort.push_back(angle_measured[3]); 
  // joint_cmd.effort.push_back(angle_measured[4]); 
  // joint_cmd.effort.push_back(angle_measured[5]);  

  joint_command_pub_.publish(joint_cmd);
}

void DasomControl::test()
{
  geometry_msgs::Twist first_publisher;
  geometry_msgs::Twist second_publisher;

  first_publisher.linear.x = EE_command_vel_limit[0];
  first_publisher.linear.y = EE_command_vel_limit[1];
  first_publisher.linear.z = EE_command_vel_limit[2];
  first_publisher.angular.x = EE_command[0];
  first_publisher.angular.y = EE_command[1];
  first_publisher.angular.z = EE_command[2];

  second_publisher.linear.x = angle_measured[0];
  second_publisher.linear.y = angle_measured[1];
  second_publisher.linear.z = angle_measured[2];
  second_publisher.angular.x = angle_measured[3];
  second_publisher.angular.y = angle_measured[4];
  second_publisher.angular.z = angle_measured[5];

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
  ROS_INFO("Start moving to initPose!!");

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
      ros::spinOnce();

      ds_ctrl_.AngleSafeFunction();
      ds_ctrl_.PublishData();
    }
    else
    {
      ds_ctrl_.CommandGenerator();
      ds_ctrl_.CommandVelocityLimit();
      ds_ctrl_.SolveInverseKinematics();
      ds_ctrl_.DOB();
      ds_ctrl_.AngleSafeFunction();
      ds_ctrl_.PublishData();
      ds_ctrl_.test();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}