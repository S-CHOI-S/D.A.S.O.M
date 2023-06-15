#ifndef TORQUE_JACOBIAN_H_
#define TORQUE_JACOBIAN_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>
#include <dynamixel_workbench_msgs/EECommand.h>

#define PI 3.14159256359

class TorqJ
{
 public:
  TorqJ();
  ~TorqJ();

  double time_loop = 0;
  double time_i = 0;
  double time_f = 0;

  double X = 0.25675;
  double Y = 0;

  double X_P_gain = 1;
  double X_I_gain = 1;
  double X_D_gain = 1;

  double Y_P_gain = 1;
  double Y_I_gain = 1;
  double Y_D_gain = 1;

  double VX_P_gain = 1;
  double VX_I_gain = 1;
  double VX_D_gain = 1;

  double VY_P_gain = 1;
  double VY_I_gain = 1;
  double VY_D_gain = 1;

//----------Link Lengths---------//
  //double Link1 = 0.12409;
  //double Link2 = 0.108;
  double Link1 = 0.10375;
  double Link2 = 0.153;

  double CoM1 = 0.08092;
  double CoM2 = 0.114;
  double delta = 0.237;
  double mass1 = 0.107;
  double mass2 = 0.296;

  double offset_1 = 0;
  double offset_2 = 0; 

  double torque_const_1 = 0;
  double torque_const_2 = 0;

  Eigen::Vector2d X_cmd;
  Eigen::Vector2d X_measured;
  Eigen::Vector2d V_measured;
  Eigen::Vector2d angle_measured;
  Eigen::Vector2d velocity_measured;
  Eigen::Vector2d effort_measured;
  Eigen::Vector2d theta_dot;

  Eigen::Vector2d Position_P_gain;
  Eigen::Vector2d Position_I_gain;
  Eigen::Vector2d Position_D_gain;

  Eigen::Vector2d Velocity_P_gain;
  Eigen::Vector2d Velocity_I_gain;
  Eigen::Vector2d Velocity_D_gain;

  Eigen::Vector2d X_error_p;
  Eigen::Vector2d X_error_p_i;
  Eigen::Vector2d X_error_i;
  Eigen::Vector2d X_error_I;
  Eigen::Vector2d X_error_d;

  // ADD
  Eigen::Vector2d X_error;
  Eigen::Vector2d pre_X_error;

  Eigen::Vector2d V_error_p;
  Eigen::Vector2d V_error_p_i;
  Eigen::Vector2d V_error_i;
  Eigen::Vector2d V_error_d;

  Eigen::Vector2d X_PID;
  Eigen::Vector2d X_PID_i;
  Eigen::Vector2d V_PID;
  Eigen::Vector2d V_PID_LPF;
  Eigen::Vector2d V_PID_LPF_i;
  Eigen::Vector2d torque_const;
  Eigen::Vector2d offset;

  Eigen::Vector2d V_test;

  Eigen::Matrix2d J;
  Eigen::Matrix2d JT;

  //Eigen::MatrixXd q_dot;
  Eigen::Vector2d tau_des;
  Eigen::Vector2d tau_des_LPF;
  Eigen::Vector2d tau_des_LPF_i;
  Eigen::Vector2d tau_gravity; //중력에 의해 조인트에 가해지는 토크
  Eigen::Vector2d tau_loop;
  Eigen::Vector2d calc_tau;

  void calc_des();
  void calc_taudes();
  void PublishCmdNMeasured();
  void ForceEstimatePre();
  void Limiter();

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  std::string robot_name_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher joint_command_pub_;
  ros::Publisher joint_measured_pub_;
  ros::Publisher test_pub_;
  ros::Subscriber EE_command_sub_;
  ros::Subscriber forwardkinematics_sub_;
  ros::Subscriber joint_states_sub_;

  double LOW_PASS_FILTER_FUNC(double Value_LPF_i, double Value_measured, double WL)
{
  //double WL = Cut_Off_Frequency * (2 * PI);
  double Value_LPF = Value_LPF_i * WL + (1 - WL) * Value_measured;

  return Value_LPF;
}
  
  static Eigen::MatrixXd EE_pos(double theta_1, double theta_2)
{
  double l1 = 0.10375;
  double l2 = 0.153;
  double cos1 = cos(theta_1);
  double cos2 = cos(theta_2);
  double sin1 = sin(theta_1);
  double sin2 = sin(theta_2);
  double cos12 = cos(theta_1 + theta_2);
  double sin12 = sin(theta_1 + theta_2);

  Eigen::MatrixXd EE_pos(2,1);

  EE_pos <<
  // X
  l1 * cos1 + l2 * cos12,
  // Y
  l1 * sin1 + l2 * sin12;

  return EE_pos;
};

  static Eigen::MatrixXd Jacobian(double theta_1, double theta_2)
{
  double l1 = 0.10375;
  double l2 = 0.153;
  double cos1 = cos(theta_1);
  double cos2 = cos(theta_2);
  double sin1 = sin(theta_1);
  double sin2 = sin(theta_2);
  double cos12 = cos(theta_1 + theta_2);
  double sin12 = sin(theta_1 + theta_2);

  Eigen::MatrixXd J(2,2);

  J <<
  // 1X1
  -l1 * sin1 - l2 * sin12,
  // 1X2
  -l2 * sin12,
  // 2X1
  l1 * cos1 + l2 * cos12,
  // 2X2
  l2 * cos12;

  return J;
};

  void poseCallback(const geometry_msgs::Twist &msg);
  void commandCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);


};

#endif //TorqJ_H_
