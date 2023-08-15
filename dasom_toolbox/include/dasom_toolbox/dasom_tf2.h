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

#ifndef DASOM_TF2_H_
#define DASOM_TF2_H_

#include <ros/ros.h>
#include <cstdio>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#define PI 3.141592

class DasomTF2
{
 public:
  DasomTF2(ros::Subscriber& subscriber, std::string topic_name, 
	   std::string parent_frame, std::string child_frame);
  ~DasomTF2();

  void test();

 private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  ros::Subscriber tf2_sub_;

  std::string parent_frame_;
  std::string child_frame_; 

  void CallbackTwist(const geometry_msgs::Twist& msg);
  void CallbackPoseStamped(const geometry_msgs::PoseStamped& msg);

};

#endif /*DASOM_TF2_H_*/