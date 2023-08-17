
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "omni_msgs/OmniButtonEvent.h"



#define PI 3.141592

geometry_msgs::Vector3 rpy; 


double roll;
double pitch;
double yaw;



void global_gimbalCallback(const geometry_msgs::Twist& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "global_gimbal_tf";
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

}

void optitrackCallback(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "paletrone";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;
    
    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation,quat);

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // ROS_WARN("R: %lf, P: %lf, Y: %lf",roll, pitch, yaw);

    br.sendTransform(transformStamped);

}


void joystickCallback(const geometry_msgs::Twist& msg)
{
//     static tf2_ros::StaticTransformBroadcaster br_joystick;
//     geometry_msgs::TransformStamped transformStamped_joystick;

//     transformStamped_joystick.header.stamp = ros::Time::now();
//     transformStamped_joystick.header.frame_id = "baseLink_from_opti";
//     transformStamped_joystick.child_frame_id = "EE_cmd_tf";
//     transformStamped_joystick.transform.translation.x = msg.linear.x;
//     transformStamped_joystick.transform.translation.y = msg.linear.y;
//     transformStamped_joystick.transform.translation.z = msg.linear.z;
    
//     tf::Quaternion quat;
//     quat.setRPY(0, 0, 0); //quat to rpy

//     transformStamped_joystick.transform.rotation.x = quat.x();
//     transformStamped_joystick.transform.rotation.y = quat.y();
//     transformStamped_joystick.transform.rotation.z = quat.z();
//     transformStamped_joystick.transform.rotation.w = quat.w();

//    // br_joystick.sendTransform(transformStamped_joystick); 일단 필요없어보임. 헷갈리니까 지우자.
}




void PoseCallback(const geometry_msgs::Twist& msg)
{//use
    static tf2_ros::StaticTransformBroadcaster br_pose;
    geometry_msgs::TransformStamped transformStamped_pose;

    transformStamped_pose.header.stamp = ros::Time::now();
    transformStamped_pose.header.frame_id = "paletrone";
    transformStamped_pose.child_frame_id = "global_EE_pose";
    transformStamped_pose.transform.translation.x = msg.linear.x;
    transformStamped_pose.transform.translation.y = msg.linear.y;
    transformStamped_pose.transform.translation.z = msg.linear.z;
    
    tf::Quaternion quat;

    quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);


    transformStamped_pose.transform.rotation.x = quat.x();
    transformStamped_pose.transform.rotation.y = quat.y();
    transformStamped_pose.transform.rotation.z = quat.z();
    transformStamped_pose.transform.rotation.w = quat.w();
    
    br_pose.sendTransform(transformStamped_pose);

    // ROS_INFO("BBBBBBBBBBBBBBBBBBBBBBB%lf %lf %lf %lf %lf %lf", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);



}


bool grey_button = true;

void gimbalTF_Callback(const geometry_msgs::PoseStamped &msg)
{  
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "global_gimbal_tf";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    br.sendTransform(transformStamped);
}


void gimbalCallback(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br_gimbal;
    geometry_msgs::TransformStamped transformStamped_pose;

    transformStamped_pose.header.stamp = ros::Time::now();
    transformStamped_pose.header.frame_id = "paletrone";
    transformStamped_pose.child_frame_id = "gimbal_tf";
    transformStamped_pose.transform.translation.x = msg.pose.position.x;
    transformStamped_pose.transform.translation.y = msg.pose.position.y;
    transformStamped_pose.transform.translation.z = msg.pose.position.z;

    transformStamped_pose.transform.rotation.x = msg.pose.orientation.x;
    transformStamped_pose.transform.rotation.y = msg.pose.orientation.y;
    transformStamped_pose.transform.rotation.z = msg.pose.orientation.z;
    transformStamped_pose.transform.rotation.w = msg.pose.orientation.w;
    br_gimbal.sendTransform(transformStamped_pose);

    // ROS_INFO("BBBBBBBBBBBBBBBBBBBBBBB%lf %lf %lf %lf %lf %lf", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);

}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"setting_joystickCMD_tf");

    ros::NodeHandle nh;
    ros::Subscriber sub_EEcmd = nh.subscribe("/dasom/EE_cmd", 10, &joystickCallback); // command EE position
    ros::Subscriber sub_optitrack = nh.subscribe("/dasombasePlate/world", 10, &optitrackCallback);
    ros::Subscriber sub_EEpose = nh.subscribe("/dasom/EE_pose", 10, &PoseCallback); //Forward kinematics
     ros::Subscriber sub_gimbal_tf = nh.subscribe("/dasom/gimbal_tf", 10, &gimbalCallback);

    ros::Publisher EE_drone_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/dasom/EE_drone_pose", 10);

    ros::Subscriber gimbal_pose_sub_ = nh.subscribe("/dasom/global_gimbal_pose", 1, gimbalTF_Callback, ros::TransportHints().tcpNoDelay());



    ros::spin();
    return 0;
}