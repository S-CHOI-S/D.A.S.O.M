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

  X_P_gain = node_handle_.param<double>("X_P_gain",1);
  X_I_gain = node_handle_.param<double>("X_I_gain",1);
  X_D_gain = node_handle_.param<double>("X_D_gain",1);

  Y_P_gain = node_handle_.param<double>("Y_P_gain",1);
  Y_I_gain = node_handle_.param<double>("Y_I_gain",1);
  Y_D_gain = node_handle_.param<double>("Y_D_gain",1);

  VX_P_gain = node_handle_.param<double>("VX_P_gain",1);
  VX_I_gain = node_handle_.param<double>("VX_I_gain",1);
  VX_D_gain = node_handle_.param<double>("VX_D_gain",1);

  VY_P_gain = node_handle_.param<double>("VY_P_gain",1);
  VY_I_gain = node_handle_.param<double>("VY_I_gain",1);
  VY_D_gain = node_handle_.param<double>("VY_D_gain",1);  

  torque_const_1 = node_handle_.param<double>("torque_const_1",1);  
  torque_const_2 = node_handle_.param<double>("torque_const_2",1);

  offset_1 = node_handle_.param<double>("offset_1",1);
  offset_2 = node_handle_.param<double>("offset_2",1);
  
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  Position_P_gain << X_P_gain, Y_P_gain;
  Position_I_gain << X_I_gain, Y_I_gain;
  Position_D_gain << X_D_gain, Y_D_gain;

  Velocity_P_gain << VX_P_gain, VY_P_gain;
  Velocity_I_gain << VX_I_gain, VY_I_gain;
  Velocity_D_gain << VX_D_gain, VY_D_gain;

  torque_const << torque_const_1, torque_const_2;
  offset << offset_1, offset_2;

  X_error_i << 0, 0;
  tau_des_LPF_i << 0, 0;
  // V_PID_LPF_i << 0, 0;
  // V_error_i << 0, 0;

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
  test_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/test", 10);
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
  // X_PID[0] = msg->position.at(0);
  // X_PID[1] = msg->position.at(1);
  // std::cout<<X_cmd<<std::endl<<"#########################################"<<std::endl;
}

void TorqJ::poseCallback(const geometry_msgs::Twist &msg)
// 현재 EE_pose 값 받아오기
{
  // X_measured[FK] 값 받아오기()
  X_measured[0] = msg.linear.x;
  X_measured[1] = msg.linear.y;
}

void TorqJ::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);

  velocity_measured[0] = msg->velocity.at(0);
  velocity_measured[1] = msg->velocity.at(1);

  effort_measured[0] = msg->effort.at(0);
  effort_measured[1] = msg->effort.at(1);

  // Jacobian 계산하기()
  J = Jacobian(angle_measured[0], angle_measured[1]);
  // Jacobian transpose()
  JT = J.transpose();

  // theta_dot << msg->velocity.at(0), msg->velocity.at(1);

  // V_measured = J * theta_dot;
}

void TorqJ::calc_des()
{
  // PID Control(위치 오차로 인한 전류값)

  // for(int i = 0; i < 2; i++)
  // {
  //   X_error_p[i] = X_cmd[i] - X_measured[i];
  //   X_error_i[i] = X_error_i[i] + X_error_p[i] * time_loop;

  //   if (X_error_p[i] - X_error_p_i[i] != 0)
  //     X_error_d[i] = (X_error_p[i] - X_error_p_i[i]) / time_loop;

  //   X_error_p_i[i] = X_error_p[i];

  //   X_PID[i] = Position_P_gain[i] * X_error_p[i] + Position_I_gain[i] * X_error_i[i] + Position_D_gain[i] * X_error_d[i];
  
  //   if (X_PID[i] - X_PID_i[i] != 0)
  //     V_PID[i] = (X_PID[i] - X_PID_i[i]) / time_loop;

  //   else V_PID[i] = 0; // 이거 혹시 몰라서 추가함

  //   X_PID_i[i] = X_PID[i];
  // }

  for(int i = 0; i < 2; i++)
  {
    X_error[i] = X_cmd[i] - X_measured[i];

    X_PID[i] = Position_P_gain[i] * X_error[i] + 
               Position_D_gain[i] * (X_error[i] - pre_X_error[i]) / time_loop;

    pre_X_error[i] = X_error[i];
  }

  // for(int i = 0; i < 2; i++)
  // {
  //   V_PID_LPF[i] = LOW_PASS_FILTER_FUNC(V_PID_LPF_i[i], V_PID[i], 0.9);
  //   V_PID_LPF_i[i] = V_PID_LPF[i];
  // }

  // tau_loop = JT * V_PID;
  tau_loop = JT * X_PID;

  ROS_WARN("tau_loop!");
  std::cout<<tau_loop<<std::endl<<"---------------------------------------"<<std::endl;
  

  // 중력 Matrix (OCM 있을 때)--------------------------------
	tau_gravity << torque_const[0] * (mass1 * CoM1 * cos(angle_measured[0]) + mass2 * Link1 * cos(angle_measured[0]) + mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1] + delta)) * 9.81 - offset[0],
	 				       torque_const[1] * (mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1] + delta)) * 9.81 - offset[1];

  ROS_WARN("tau_gravity!");
  std::cout<<tau_gravity<<std::endl<<"***************************************"<<std::endl;
}

void TorqJ::calc_taudes()
{
  tau_des = tau_loop + tau_gravity;

  // 이거 현재 안 쓰고 있는 상황
  // for(int i = 0; i<2; i++)
  // {   
  //   tau_des_LPF[i] = LOW_PASS_FILTER_FUNC(tau_des_LPF_i[i], tau_des[i], 0.016);
  //   tau_des_LPF_i[i] = tau_des_LPF[i];
  // }


  ROS_WARN("tau_des!");
  // std::cout<<JT<<std::endl<<"---------------------------------------"<<std::endl;
  // std::cout<<V_PID<<std::endl<<"***************************************"<<std::endl;
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
  joint_cmd.velocity.push_back(X_PID[0]); // cartesian space
  joint_cmd.velocity.push_back(X_PID[1]); // cartesian space
  joint_cmd.effort.push_back(tau_des[0]); // joint space
  joint_cmd.effort.push_back(tau_des[1]); // joint space
  joint_command_pub_.publish(joint_cmd);

  joint_measured.header.stamp = ros::Time::now();
  joint_measured.position.push_back(X_measured[0]);
  joint_measured.position.push_back(X_measured[1]);
  joint_measured.velocity.push_back(V_measured[0]);
  joint_measured.velocity.push_back(V_measured[1]);
  joint_measured.effort.push_back(tau_gravity[0]);
  joint_measured.effort.push_back(tau_gravity[1]);
  joint_measured_pub_.publish(joint_measured);
}

void TorqJ::ForceEstimatePre()
{
  geometry_msgs::Twist test;

  calc_tau[0] = effort_measured[0] - tau_gravity[0];
  calc_tau[1] = effort_measured[1] - tau_gravity[1];

  test.linear.x = calc_tau[0];
  test.linear.y = effort_measured[0];
  test.linear.z = tau_gravity[0];
  test.angular.x = calc_tau[1];
  test.angular.y = effort_measured[1];
  test.angular.z = tau_gravity[1];

  // ROS_WARN("update!");
  // ROS_ERROR("calc_tau");
  // std::cout<<calc_tau<<std::endl<<"***************************************"<<std::endl;
  // ROS_ERROR("effort_measured");
  // std::cout<<effort_measured<<std::endl<<"---------------------------------------"<<std::endl;
  // ROS_ERROR("tau_gravity");
  // std::cout<<tau_gravity<<std::endl<<"#########################################"<<std::endl;

  test_pub_.publish(test);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(250);

  torqJ.V_error_i << 0, 0;
  torqJ.V_PID_LPF_i << 0, 0;

  // start time
  torqJ.time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
    // time_loop update
    torqJ.time_f = ros::Time::now().toSec();
		torqJ.time_loop = torqJ.time_f - torqJ.time_i;
		torqJ.time_i = ros::Time::now().toSec();

    torqJ.calc_des();
    torqJ.calc_taudes();
    torqJ.PublishCmdNMeasured();
    torqJ.ForceEstimatePre();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}