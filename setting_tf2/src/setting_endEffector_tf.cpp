#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define PI 3.141592

geometry_msgs::Vector3 rpy; 

Eigen::VectorXd manipulator_initPose;
Eigen::VectorXd joystick_initPose;



void joystickCallback(const geometry_msgs::Twist& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "EE_tf";
    transformStamped.transform.translation.x = msg.linear.x;
    transformStamped.transform.translation.y = msg.linear.y;
    transformStamped.transform.translation.z = msg.linear.z;
    
    tf::Quaternion quat;

    quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z); //quat to rpy


    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_endEffector_tf");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/dasom/EE_pose", 10, &joystickCallback); //나바

    ros::spin();
    return 0;
}
