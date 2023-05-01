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
#include <test/Torqbian.h>
#include <dynamixel_workbench_msgs/EECommand.h>

#define PI 3.14159256359

class TorqJ
{
 public:
  TorqJ();
  ~TorqJ();

  double X = 0.23209;
  double Y = 0;

  double x_gain = 1;
  double y_gain = 1;

  double X_vel_gain = 1;
  double Y_vel_gain = 1;

  double command_x_position = 0;
  double command_y_position = 0;

  double position_x_dot = 0;
  double position_y_dot = 0;

  double measured_x_position = 0;
  double measured_y_position = 0;

  double pre_measured_x_position = 0;
  double pre_measured_y_position = 0;

  double velocity_x_dot = 0;
  double velocity_y_dot = 0;

  double measured_x_velocity = 0;
  double measured_y_velocity = 0;

//----------Link Lengths---------//
	double Link1 = 0.12409;
	double Link2 = 0.108;

  //----------Link Lengths---------//
	double CoM1 = 0.08092;
	double CoM2 = 0.114;
	double delta = 0.237;
	double mass1 = 0.107;
	double mass2 = 0.296;

  double offset_1 = 0;
  double offset_2 = 0; 

  Eigen::Vector2d X_cmd;
  Eigen::Vector2d X_dot;
  Eigen::Vector2d X_measured;
  Eigen::Vector2d X_gain;
  Eigen::Matrix2d J;
  Eigen::Matrix2d JT;
  Eigen::Vector2d EE_position;
  Eigen::Vector2d theta_dot;
  Eigen::Vector2d V_dot;
  Eigen::Vector2d V_measured;
  Eigen::Vector2d V_gain;

  //Eigen::MatrixXd q_dot;
  Eigen::Vector2d tau_des;
  Eigen::Vector2d Tau_gravity; //중력에 의해 조인트에 가해지는 토크

  //V_gain << 1,1;

  void calc_des();
  void calc_taudes();
  void PublishCmdNMeasured();

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
  ros::Subscriber EE_command_sub_;
  ros::Subscriber forwardkinematics_sub_;
  ros::Subscriber joint_states_sub_;

  
    static Eigen::MatrixXd EE_pos(double theta_1, double theta_2)
{
    double l1 = 0.12409;
    double l2 = 0.108;
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
    double l1 = 0.12409;
    double l2 = 0.108;
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

  //void EEpositionCallback(const dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res);
  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //TorqJ_H_
