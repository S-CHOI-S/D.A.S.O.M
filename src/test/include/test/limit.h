#ifndef LIMIT_H_
#define LIMIT_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>

#define PI 3.14159256359

class Limit
{
 public:
  Limit();
  ~Limit();
 void goaljointposition();

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
  double x = 0;
  double y = 0;
  double z = 0;
  double r = 0;
  double q1 = 0;
  double q2 = 0;
  double q3 = 0;
  double q4 = 0;
  double x2 = 0;
  double y2 = 0;
  double z2 = 0;
  double l1 = 0.04233;
  double l2 = 0.12409;
  double l3 = 0.12409;
  double l4 = 0.14909;
  double k1 = 0;
  double k2 = 0;
  double cos_q2 = 0;
  double sin_q2 = 0;
  double cos_q3 = 0;
  double sin_q3 = 0;
  double phi = PI/2;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher dasom_joint_states_pub_;
  ros::Subscriber goal_joint_states_sub_;

  void poseCallback(const geometry_msgs::Twist &msg);

  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //LIMIT_H_
