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
  // ros::Publisher dasom_joint_states_pub_;
  // ros::Subscriber goal_joint_states_sub_;
  ros::Publisher testerpub;
  ros::Publisher forcepub;
  ros::Subscriber effortsub;
  ros::Subscriber xysub;

  // void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void EffortToTorque(const sensor_msgs::JointState::ConstPtr &msg);
  void XYCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

};

#endif //Force_H_
