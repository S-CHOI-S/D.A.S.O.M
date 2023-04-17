#ifndef TORQUE_JACOBIAN_H_
#define TORQUE_JACOBIAN_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <test/Torqbian.h>
#include <std_msgs/String.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>
#include <cmath>
#include <dynamixel_workbench_msgs/EECommand.h>
#include <dynamixel_workbench_msgs/JointCommand.h>


#define PI 3.14159256359

class TorqJ
{
 public:
  TorqJ();
  ~TorqJ();

  double x = 0.23209; // 초기 위치 맞춰주기 -> theta1과 theta2의 각도가 0일 때로!
  double y = 0;
  double theta1 = 0;
  double theta2 = 0;
  double X_gain = 0;
  double Y_gain = 0;
  double command_x_position = 0;
  double command_y_position = 0;
  double position_x_dot = 0;
  double position_y_dot = 0;
  double measured_x_position = 0;
  double measured_y_position = 0;

  //void Jacobian_twolink();
  void commandCallback(const sensor_msgs::JointState::ConstPtr &msg);

    Eigen::MatrixXd JT;
  Eigen::VectorXd X_dot;
  Eigen::VectorXd Theta_dot;

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
  void initServer();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  //ros::Publisher twolink_states_pub;
  // ros::Subscriber twolink_joint_states_sub;
  ros::Subscriber joint_states_sub;
  // ros::Subscriber jacobian_sub;
  //ros::ServiceServer EE_command_server;
  //ros::ServiceClient client;

  bool EEpositionCallback(dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res);

  // void ForwardKinematics_twolink(const sensor_msgs::JointState::ConstPtr &msg);

//  static Eigen::Matrix4d L1(double theta_1)
//{
//    double l1 = 0.12409;
//    double cosT = cos(theta_1), sinT = sin(theta_1);
//
//    Eigen::Matrix4d L;
//
//    L <<         1,          0,         0,         0,
//                 0,       cosT,     -sinT,   l1*cosT,
//                 0,       sinT,      cosT,   l1*sinT,
//                 0,          0,         0,         1;
//    return L;
//};

//  static Eigen::Matrix4d L2(double theta_2)
//{
//    double l2 = 0.108;
//    double cosT = cos(theta_2), sinT = sin(theta_2);
//
//    Eigen::Matrix4d L;
//
//    L <<         1,          0,         0,         0,
//                 0,       cosT,     -sinT,   l2*cosT,
//                 0,       sinT,      cosT,   l2*sinT,
//                 0,          0,         0,         1;
//    return L;
//};

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



  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);



#endif //TorqJ_H_
