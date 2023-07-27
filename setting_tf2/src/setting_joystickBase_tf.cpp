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



int main(int argc, char **argv){
    ros::init(argc,argv,"setting_joystickBase_tf");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;



    // Eigen::VectorXd manipulator_initPose;
    // manipulator_initPose.resize(6,1);
    // manipulator_initPose << 0, 0.25, 0.3, 0, 0, 0;
    Eigen::VectorXd joystick_initPose;
    joystick_initPose.resize(6,1);
    //joystick_initPose << 0, 0.085595, -0.097945, 0, 1.206859, -1.674547;
    joystick_initPose << 0, 0.085595, -0.097945 , 0, 0, 0;
//-0.060348, -0.004343, 0.115148, -1.202008, 0.001187, 1.644898



    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id="world";
    static_transformStamped.child_frame_id="joystickBase";
    static_transformStamped.transform.translation.x = -joystick_initPose[0];
    static_transformStamped.transform.translation.y = -joystick_initPose[1];
    static_transformStamped.transform.translation.z = -joystick_initPose[2];
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x=quat.x();
    static_transformStamped.transform.rotation.y=quat.y();
    static_transformStamped.transform.rotation.z=quat.z();
    static_transformStamped.transform.rotation.w=quat.w();

    static_broadcaster.sendTransform(static_transformStamped);
    ros::spin();
    return 0;
}