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

DasomTF2::DasomTF2(ros::Subscriber& subscriber, std::string topic_name, std::string parent_frame, std::string child_frame)
: nh_(""), loop_rate_(200)
{
  tf2_sub_ = subscriber;

  if(!(topic_name.c_str() == "/dasombasePlate/world")) tf2_sub_ = nh_.subscribe(topic_name.c_str(),10,&DasomTF2::CallbackTwist,this);
  else tf2_sub_ = nh_.subscribe(topic_name.c_str(),10,&DasomTF2::CallbackPoseStamped,this);
  
  parent_frame_ = parent_frame;
  child_frame_ = child_frame;
}

DasomTF2::~DasomTF2()
{
  
}

void DasomTF2::test()
{
  ROS_INFO("HI! Here is DasomTF2!");
}

void DasomTF2::CallbackTwist(const geometry_msgs::Twist& msg)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame_.c_str();
  transformStamped.child_frame_id = child_frame_.c_str();
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

  ros::spinOnce();
}

void DasomTF2::CallbackPoseStamped(const geometry_msgs::PoseStamped& msg)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame_.c_str();
  transformStamped.child_frame_id = child_frame_.c_str();
  transformStamped.transform.translation.x = msg.pose.position.x;
  transformStamped.transform.translation.y = msg.pose.position.y;
  transformStamped.transform.translation.z = msg.pose.position.z;

  transformStamped.transform.rotation.x = msg.pose.orientation.x;
  transformStamped.transform.rotation.y = msg.pose.orientation.y;
  transformStamped.transform.rotation.z = msg.pose.orientation.z;
  transformStamped.transform.rotation.w = msg.pose.orientation.w;

  br.sendTransform(transformStamped);

  ros::spinOnce();
}