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
  DasomTF2(ros::Subscriber& subscriber, std::string str);
  ~DasomTF2();

  ros::Subscriber tf2_sub_;

  void test();
  void Callback(const geometry_msgs::Twist& msg);

 private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;


};

#endif /*DASOM_TF2_H_*/
