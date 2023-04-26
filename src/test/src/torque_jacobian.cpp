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
  // std::cout<<V_gain<<std::endl<<"---------------------------------------"<<std::endl;

  ROS_INFO("TorqJ node start");
}

TorqJ::~TorqJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void TorqJ::initPublisher()
{
  joint_command_pub = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
  // joint_command_pub = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
}

void TorqJ::initSubscriber()
{
  joint_states_sub  = node_handle_.subscribe("/joint_states", 10, &TorqJ::poseCallback, this);
  EE_command_sub = node_handle_.subscribe("/goal_EE_position", 10, &TorqJ::commandCallback, this);
  // dasom_joint_states_sub = node_handle_.subscribe(robot_name_ + "/joint_states", 10, &TorqJ::poseCallback, this);
}

void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
// cmd_position 값 받아서 X'구하기
{
  // X_command, Y_command 값 받아오기(O)
  X_cmd[0] = msg->position.at(0);
  X_cmd[1] = msg->position.at(1);

  // X_dot 계산하기(O)
	X_dot[0] = X_gain[0] * (X_cmd[0] - X_measured[0]);
  X_dot[1] = X_gain[1] * (X_cmd[1] - X_measured[1]);

  // V_dot 계산하기(O)
  V_dot[0] = V_gain[0] * (X_dot[0] - V_measured[0]);
  V_dot[1] = V_gain[1] * (X_dot[1] - V_measured[1]);

  // V_dot << velocity_x_dot, velocity_y_dot;

  // std::cout<<V_dot<<std::endl<<"---------------------------------------"<<std::endl;
  // std::cout<<V_dot<<std::endl<<"---------------------------------------"<<std::endl;
  
  // ROS_INFO("command_position_fromGUI = %lf, p_gain = %lf, position_dot = %lf", command_position_fromGUI, p_gain, position_dot);
  // ROS_INFO("x'= %lf, y'= %lf, z'= %lf",position_x_dot, position_y_dot, position_z_dot);
  // ROS_WARN("r'= %lf, p'= %lf, y'= %lf",angle_r_dot, angle_p_dot, angle_yaw_dot);
}

void TorqJ::poseCallback(const sensor_msgs::JointState::ConstPtr &msg)
// 현재 joint angle 값 받아오기
{
  // EE_position 계산하기 = X_measured(O)
  X_measured = EE_pos(msg->position.at(0), msg->position.at(1));

  // Jacobian 계산하기(O)
  J = Jacobian(msg->position.at(0),msg->position.at(1));
  // Jacobian transpose(O)
  JT = J.transpose();

  theta_dot << msg->velocity.at(0), msg->velocity.at(1);

  V_measured = J * theta_dot;

  // std::cout<<theta_dot<<std::endl<<"---------------------------------------"<<std::endl;
  // std::cout<<V_measured<<std::endl<<"---------------------------------------"<<std::endl;
}

void TorqJ::calc_qdot()
{
  sensor_msgs::JointState joint_cmd;

  JT.resize(2,2);
  V_dot.resize(2,1);
  
  tau_des = JT * V_dot;

  std::cout<<JT<<std::endl<<"---------------------------------------"<<std::endl;
  std::cout<<V_dot<<std::endl<<"***************************************"<<std::endl;
  std::cout<<tau_des<<std::endl<<"#########################################"<<std::endl;

  // tau_des를 publish
  joint_cmd.effort.push_back(tau_des[0]);
  joint_cmd.effort.push_back(tau_des[1]);
  joint_command_pub.publish(joint_cmd);
  
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(250); //250
  while (ros::ok())
  {
    torqJ.calc_qdot();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}