
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
// #include "dasom_toolbox/dasom_lpf.h"


#define PI 3.141592

double roll;
double pitch;
double yaw;




Eigen::VectorXd optitrackQuat;
Eigen::VectorXd optitrackQuat_lpf;


void optitrackCallback22(const geometry_msgs::PoseStamped& msg)
{
    //use
    optitrackQuat[0] = msg.pose.position.x - 0.091430;
    optitrackQuat[1] = msg.pose.position.y - 0.058208;
    optitrackQuat[2] = msg.pose.position.z - 0.012032;
    
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    roll -= 0.188241;
    pitch -= -1.539353;
    yaw -= -1.770787;

    quat.setRPY(roll, pitch, yaw);

    optitrackQuat[3] = quat.x();
    optitrackQuat[4] = quat.y();
    optitrackQuat[5] = quat.z();
    optitrackQuat[6] = quat.w();
}





// void optitrackCallback(const geometry_msgs::PoseStamped& msg)
// {//use
//     static tf2_ros::StaticTransformBroadcaster br;
//     geometry_msgs::TransformStamped transformStamped;

//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "world";
//     transformStamped.child_frame_id = "paletrone";
//     transformStamped.transform.translation.x = msg.pose.position.x;
//     transformStamped.transform.translation.y = msg.pose.position.y;
//     transformStamped.transform.translation.z = msg.pose.position.z;
    
//     transformStamped.transform.rotation.x = msg.pose.orientation.x;
//     transformStamped.transform.rotation.y = msg.pose.orientation.y;
//     transformStamped.transform.rotation.z = msg.pose.orientation.z;
//     transformStamped.transform.rotation.w = msg.pose.orientation.w;

//     br.sendTransform(transformStamped);
// }



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
}


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
}



int main(int argc, char **argv)
{


    ros::init(argc,argv,"setting_joystickCMD_tf");
    ros::NodeHandle nh;

    // ros::Subscriber sub_optitrack = nh.subscribe("/dasombasePlate/world", 10, &optitrackCallback);
    ros::Subscriber sub_EEpose = nh.subscribe("/dasom/EE_pose", 10, &PoseCallback); 
    ros::Subscriber sub_gimbal_tf = nh.subscribe("/dasom/gimbal_tf", 10, &gimbalCallback);
    ros::Subscriber gimbal_pose_sub_ = nh.subscribe("/dasom/global_gimbal_pose", 1, gimbalTF_Callback, ros::TransportHints().tcpNoDelay());

    ros::Publisher EE_drone_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/dasom/EE_drone_pose", 10);




    ros::Subscriber sub_optitrack2 = nh.subscribe("/dasombasePlate/world", 10, &optitrackCallback22);


    ros::Rate loop(100);
    double time_i = ros::Time::now().toSec();
    double time_f = 0;
    double time_loop = 0;

    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    optitrackQuat.resize(7);
    optitrackQuat_lpf.resize(7);


    // DasomLPF ds1(8);
    // DasomLPF ds2(8);
    // DasomLPF ds3(8);
    // DasomLPF ds4(8);
    // DasomLPF ds5(8);
    // DasomLPF ds6(8);
    // DasomLPF ds7(8);


    while(ros::ok())
    {
    time_f = ros::Time::now().toSec();
    time_loop = time_f - time_i;
    time_i = ros::Time::now().toSec();

    // optitrackQuat_lpf[0] = ds1.updateLPF(time_loop, optitrackQuat[0]);
    // optitrackQuat_lpf[1] = ds2.updateLPF(time_loop, optitrackQuat[1]);
    // optitrackQuat_lpf[2] = ds3.updateLPF(time_loop, optitrackQuat[2]);
    // optitrackQuat_lpf[3] = ds4.updateLPF(time_loop, optitrackQuat[3]);
    // optitrackQuat_lpf[4] = ds5.updateLPF(time_loop, optitrackQuat[4]);
    // optitrackQuat_lpf[5] = ds6.updateLPF(time_loop, optitrackQuat[5]);
    // optitrackQuat_lpf[6] = ds7.updateLPF(time_loop, optitrackQuat[6]);


        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "paletrone";
        transformStamped.transform.translation.x = optitrackQuat[0];
        transformStamped.transform.translation.y = optitrackQuat[1];
        transformStamped.transform.translation.z = optitrackQuat[2];

        transformStamped.transform.rotation.x = optitrackQuat[3];
        transformStamped.transform.rotation.y = optitrackQuat[4];
        transformStamped.transform.rotation.z = optitrackQuat[5];
        transformStamped.transform.rotation.w = optitrackQuat[6];

        br.sendTransform(transformStamped);

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}