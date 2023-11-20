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

#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "omni_msgs/OmniButtonEvent.h"
#include "dasom_toolbox/dasom_joint.h"

#define PI 3.14159256359

class TFBroadcaster
{
 public:
  TFBroadcaster();
  ~TFBroadcaster();

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For time loop (count)
  double time_i = 0;
  double time_f = 0;
  double time_loop = 0;

  // For angle control
  Eigen::VectorXd angle_ref;
  Eigen::VectorXd angle_measured;

  // For optitrack lpf
  Eigen::VectorXd optitrackQuat;
  Eigen::VectorXd optitrackQuat_lpf; 


  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void palletroneOptitrackLPF(geometry_msgs::PoseStamped transformstamped);

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  std::string robot_name_;

  /*****************************************************************************
  ** D.A.S.O.M toolbox
  *****************************************************************************/
  // DasomJoint
  DasomJoint *ds_jnt_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initPublisher();

  /*****************************************************************************
  ** ROS Publishers
  *****************************************************************************/

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  // ros::Subscriber test_sub_;
  // ros::Subscriber joystick_pose_sub_;

  // ros::Subscriber sub_gimbal_tf_;
  //                                            
  ros::Subscriber palletrone_optitrack_sub_;
  ros::Subscriber dasom_EE_pose_sub_; 
  ros::Subscriber global_fixed_gimbal_pose_sub_;  
  ros::Subscriber dasom_EE_cmd_sub_;

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For optitrack lpf
  // Eigen::VectorXd optitrackQuat;
  // Eigen::VectorXd optitrackQuat_lpf;   

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/

  void palletroneOptitrackCallback(const geometry_msgs::PoseStamped& msg);
  void dasomEEPoseCallback(const geometry_msgs::Twist& msg);
  void dasomEECmdCallback(const geometry_msgs::Twist& msg);
  void globalFixedGimbalPoseCallback(const geometry_msgs::PoseStamped &msg);
};

#endif //TF_BROADCASTER_H_