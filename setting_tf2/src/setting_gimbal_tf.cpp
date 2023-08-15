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


void joystickCallback(const geometry_msgs::Twist& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "gimbal_tf";
    transformStamped.transform.translation.x = msg.linear.x;
    transformStamped.transform.translation.y = msg.linear.y;
    transformStamped.transform.translation.z = msg.linear.z;
    
        tf2::Quaternion quat;
        quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    ROS_INFO("a");

    br.sendTransform(transformStamped);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_gimbal_tf");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/dasom/gimbal_tf", 10, &joystickCallback);


    ros::spin();
    return 0;
}