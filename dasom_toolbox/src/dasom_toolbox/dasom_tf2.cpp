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

#include "../../include/dasom_toolbox/dasom_tf2.h"

DasomTF2::DasomTF2(ros::Subscriber& subscriber, std::string str)
: nh_(""), loop_rate_(200)
{
  tf2_sub_ = subscriber;
  ROS_WARN("%s",str.c_str());
  tf2_sub_ = nh_.subscribe(str.c_str,10,&DasomTF2::Callback,this);
}

DasomTF2::~DasomTF2()
{
  
}

void DasomTF2::test()
{
  while(ros::ok())
  {
    ROS_INFO("HI! Here is DasomTF2!");
  }
}

void DasomTF2::Callback(const geometry_msgs::Twist& msg)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "joystickCMD";
  transformStamped.transform.translation.x = msg.linear.x;
  transformStamped.transform.translation.y = msg.linear.y;
  transformStamped.transform.translation.z = msg.linear.z;
  
  tf::Quaternion quat;

  quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();
  
  br.sendTransform(transformStamped);

  ros::spin();
}