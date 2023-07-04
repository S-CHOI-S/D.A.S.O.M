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
#include <two_link/Torqbian.h>
#include <dynamixel_workbench_msgs/EECommand.h>

#define PI 3.14159256359

class TorqJ
{
 public:
  TorqJ();
  ~TorqJ();

  double X = 0.23209;
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

  double torque_const_1 = 0;
  double torque_const_2 = 0;

  Eigen::Vector2d X_cmd;
  Eigen::Vector2d X_measured;
  Eigen::Vector2d V_measured;
  Eigen::Vector2d angle_measured;
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
  Eigen::Vector2d X_error_d;

  Eigen::Vector2d V_error_p;
  Eigen::Vector2d V_error_p_i;
  Eigen::Vector2d V_error_i;
  Eigen::Vector2d V_error_d;

  Eigen::Vector2d X_PID;
  Eigen::Vector2d V_PID;
  Eigen::Vector2d torque_const;
  Eigen::Vector2d offset;


  Eigen::Matrix2d J;
  Eigen::Matrix2d JT;

  //Eigen::MatrixXd q_dot;
  Eigen::Vector2d tau_des;
  Eigen::Vector2d tau_gravity; //중력에 의해 조인트에 가해지는 토크
  Eigen::Vector2d tau_loop;
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
  sensor_msgs::JointState command_position;
  geometry_msgs::Twist Measured_EE_Position;
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

  void poseCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void commandCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);

  //void EEpositionCallback(const dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res);
  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //TorqJ_H_

