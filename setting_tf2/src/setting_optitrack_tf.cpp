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

#define PI 3.141592

geometry_msgs::Vector3 rpy; 


Eigen::Vector3d joystick_oriscribed;


void joystickCallback(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "baseLink_from_opti";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;
    
    tf::Quaternion quat;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;


    br.sendTransform(transformStamped);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_optitrack_tf");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/dasombasePlate/world", 10, &joystickCallback);


    ros::spin();
    return 0;
}