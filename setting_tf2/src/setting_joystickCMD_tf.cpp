#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define PI 3.141592

geometry_msgs::Vector3 rpy; 

Eigen::Vector3d manipulator_initPose;
Eigen::Vector3d joystick_initPose;
Eigen::Vector3d joystick_subscribed;


void joystickCallback(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joystickBase";
    transformStamped.child_frame_id = "joystickCMD";
    transformStamped.transform.translation.x = msg.pose.position.x + manipulator_initPose[0];
    transformStamped.transform.translation.y = msg.pose.position.y + manipulator_initPose[1];
    transformStamped.transform.translation.z = msg.pose.position.z + manipulator_initPose[2];
    
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(joystick_subscribed[0], joystick_subscribed[1], joystick_subscribed[2]); //quat to rpy

    joystick_subscribed -= joystick_initPose;

    quat.setRPY(joystick_subscribed[0], joystick_subscribed[1], joystick_subscribed[2]);

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_joystickCMD_tf");
    joystick_initPose << 0, 1.206859, -1.674547;
    manipulator_initPose << 0, 0.25, 0.3;

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/phantom/pose", 10, &joystickCallback);


    ros::spin();
    return 0;
}