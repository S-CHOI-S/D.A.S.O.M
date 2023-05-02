#ifndef FORCE_H_
#define FORCE_H_

#include <ros/ros.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <open_manipulator_msgs/KinematicsPose.h>

#define PI 3.141592

class Force
{
 public:
  Force();
  ~Force();

  Eigen::Vector2d angle_measured;
  Eigen::Vector2d velocity_measured;
  Eigen::Vector2d effort_measured;

  Eigen::Matrix2d M;
  Eigen::Vector2d C;
  Eigen::Vector2d G;

  void setDynamicsMatrix();

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
  // ros::Publisher testerpub;
  ros::Publisher estimated_force_pub_;
  ros::Subscriber joint_effort_sub_;

  static Eigen::Matrix2d Moment(double theta1, double theta2)
{
  double l1 = 0.12409;
  double l2 = 0.108;
  double m1 = 0.107;
  double m2 = 0.296;
  double c1 = cos(theta1);
  double c2 = cos(theta2);

  Eigen::Matrix2d M;

  M <<
  // 1X1
  pow(l2,2) * m2 + 2 * l1 * l2 * m2 * c2 + pow(l1,2) * (m1 + m2),
  // 1X2
  pow(l2,2) * m2 + l1 * l2 * m2 * c2,
  // 2X1
  pow(l2,2) * m2 + l1 * l2 * m2 * c2,
  // 2X2
  pow(l2,2) * m2;

  return M;
};

  static Eigen::Vector2d CentrifugalNCoriolis(double theta1, double theta2, 
					      double velocity1, double velocity2)
{
  double l1 = 0.12409;
  double l2 = 0.108;
  double m1 = 0.107;
  double m2 = 0.296;
  double s1 = sin(theta1);
  double s2 = sin(theta2);

  Eigen::Vector2d C;

  C <<
  // 1X1
  -m2 * l1 * l2 * s2 * pow(velocity2,2) - 2 * m2 * l1 * l2 * s2 * velocity1 * velocity2,
  // 1X2
  m2 * l1 * l2 * s2 * pow(velocity1,2);

  return C;
};

  static Eigen::Vector2d Gravity(double theta1, double theta2)
{
  double l1 = 0.12409;
  double l2 = 0.108;
  double m1 = 0.107;
  double m2 = 0.296;
  double g = -9.81; ///////////////// 이거 (-) 맞는 지 한 번 확인해야!
  double c1 = cos(theta1);
  double c12 = cos(theta1 + theta2);

  Eigen::Vector2d G;

  G <<
  // 1X1
  m2 * l2 * g * c12 + (m1 + m2) * l1 * g * c1,
  // 1X2
  m2 * l2 * g * c12;

  return G;
};

  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //Force_H_
