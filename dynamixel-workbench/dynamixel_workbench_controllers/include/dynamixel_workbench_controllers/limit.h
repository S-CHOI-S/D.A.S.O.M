#ifndef LIMIT_H_
#define LIMIT_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>

class Limit
{
 public:
  Limit();
  ~Limit();
 

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
  ros::Publisher dasom_joint_states_pub_;
  ros::Subscriber goal_joint_states_sub_;

  void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //LIMIT_H_
