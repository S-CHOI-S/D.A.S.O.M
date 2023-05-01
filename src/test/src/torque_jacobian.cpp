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

#include "test/torque_jacobian.h"

TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  x_gain = node_handle_.param<int>("X_gain",1);
  y_gain = node_handle_.param<int>("Y_gain",1);

  X_vel_gain = node_handle_.param<int>("X_vel_gain",1);
  Y_vel_gain = node_handle_.param<int>("Y_vel_gain",1);
  
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  X_gain << x_gain, y_gain;
  V_gain << X_vel_gain, Y_vel_gain;

  ROS_INFO("TorqJ node start");
}

TorqJ::~TorqJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void TorqJ::initPublisher()
{
  joint_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
  joint_measured_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/measured_dynamixel_position", 10);
}

void TorqJ::initSubscriber()
{
  EE_command_sub_ = node_handle_.subscribe("/goal_EE_position", 10, &TorqJ::commandCallback, this);
  forwardkinematics_sub_ = node_handle_.subscribe("/EE_pose", 10, &TorqJ::poseCallback, this);
  joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &TorqJ::jointCallback, this);
}

void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
// cmd_position 값 받아서 X'구하기
{
  // X_command, Y_command 값 받아오기(O)
  X_cmd[0] = msg->position.at(0);
  X_cmd[1] = msg->position.at(1);
}

void TorqJ::poseCallback(const geometry_msgs::Twist &msg)
// 현재 EE_pose 값 받아오기
{
  // X_measured[FK] 값 받아오기()
  X_measured[0] = msg.linear.x;
  X_measured[1] = msg.linear.y;

	// // 중력 매트릭스 (OCM 있을 때)--------------------------------
	// Tau_gravity << (mass1 * CoM1 * cos(msg->position.at(0)) + mass2 * Link1 * cos(msg->position.at(0)) + mass2 * CoM2 * cos(msg->position.at(0) + msg->position.at(1))) * 9.81 - offset_1,
	// 				mass2 * CoM2 * cos(msg->position.at(0) + msg->position.at(1)) * 9.81 - offset_2;
}

void TorqJ::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Jacobian 계산하기()
  J = Jacobian(msg->position.at(0),msg->position.at(1));
  // Jacobian transpose()
  JT = J.transpose();
  ///////////////////////////////////////////////////////////////////////
  theta_dot << msg->velocity.at(0), msg->velocity.at(1);

  V_measured = J * theta_dot;
  /////////////////////////////////////////////////////////////////////// 이 정도 계산은 어떤지?
}

void TorqJ::calc_des()
{
  // X_dot 계산하기(O)
	X_dot[0] = X_gain[0] * (X_cmd[0] - X_measured[0]);
  X_dot[1] = X_gain[1] * (X_cmd[1] - X_measured[1]);

  // V_dot 계산하기(O)
  V_dot[0] = V_gain[0] * (X_dot[0] - V_measured[0]);
  V_dot[1] = V_gain[1] * (X_dot[1] - V_measured[1]);
}


void TorqJ::calc_taudes()
{
  JT.resize(2,2);
  V_dot.resize(2,1);
  
  tau_des = JT * V_dot;// + Tau_gravity*0.01;
  ROS_WARN("update!");
  std::cout<<JT<<std::endl<<"---------------------------------------"<<std::endl;
  std::cout<<V_dot<<std::endl<<"***************************************"<<std::endl;
  std::cout<<tau_des<<std::endl<<"#########################################"<<std::endl;
}

void TorqJ::PublishCmdNMeasured()
{
  sensor_msgs::JointState joint_cmd;
  sensor_msgs::JointState joint_measured;

  // tau_des를 publish
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.position.push_back(X_cmd[0]); // cartesian space
  joint_cmd.position.push_back(X_cmd[1]); // cartesian space
  joint_cmd.velocity.push_back(X_dot[0]); // cartesian space
  joint_cmd.velocity.push_back(X_dot[1]); // cartesian space
  joint_cmd.effort.push_back(tau_des[0]); // joint space
  joint_cmd.effort.push_back(tau_des[1]); // joint space
  joint_command_pub_.publish(joint_cmd);

  joint_measured.header.stamp = ros::Time::now();
  joint_measured.position.push_back(X_measured[0]);
  joint_measured.position.push_back(X_measured[1]);
  joint_measured.velocity.push_back(V_measured[0]);
  joint_measured.velocity.push_back(V_measured[1]);
  joint_measured_pub_.publish(joint_measured);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    torqJ.calc_des();
    torqJ.calc_taudes();
    torqJ.PublishCmdNMeasured();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}