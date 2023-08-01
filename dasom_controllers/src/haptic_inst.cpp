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

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <geometry_msgs/Twist.h>

#define PI 3.14159256359

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "haptic_inst");
    ros::NodeHandle nh;
    ros::Publisher haptic_cmd_ = nh.advertise<geometry_msgs::Twist>("/instead_haptic",10);

    ros::Rate loop_rate(250);

    geometry_msgs::Twist msg;
    double i = 0;

    while(ros::ok())
    {
        msg.linear.x = 0 +0.1*sin(i/1000);
        msg.linear.y = -0.05; // y축 방향으로 안정적인 거 확인!
        msg.linear.z = 0; // z축 방향으로 안정적인 거 확인!
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        haptic_cmd_.publish(msg);

        i++;

        ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}