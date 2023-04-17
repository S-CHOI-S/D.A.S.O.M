#include "ros/ros.h"
#include <iostream>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_srvs/Empty.h"

#define PI 3.14159256359

double x = 0;
double y = 0;
double z = 0;
double r = 0;
double q1 = 0;
double q2 = 0;
double q3 = 0;
double q4 = 0;
double x2 = 0;
double y2 = 0;
double z2 = 0;
double l1 = 0.04233; // m 단위
double l2 = 0.12409;
double l3 = 0.12409;
double l4 = 0.14909;
double k1 = 0;
double k2 = 0;
double cos_q2;
double sin_q2;
double cos_q3;
double sin_q3;
double phi = PI/2;

double Request_Receive = 0; // ping
double Response_Send = 0; // ping

geometry_msgs::Twist angle;

void poseCallback(const geometry_msgs::Twist &msg)
{
    x = msg.linear.x;
    y = msg.linear.y;
    z = msg.linear.z;

    q1 = atan2(y,x);
    r = sqrt(pow(x,2) + pow(y,2));

    x2 = x - l4 * cos(phi) * cos(q1);
    y2 = y - l4 * cos(phi) * sin(q1);
    z2 = z - l4 * sin(phi);
    cos_q3 = (pow(x2,2) + pow(y2,2) + pow(z2 - l1,2) - pow(l2,2) - pow(l3,2)) / 2 * l2 * l3;
    sin_q3 = sqrt(1 - pow(cos_q3,2));
    q3 = atan2(sin_q3,cos_q3);
    k1 = l2 + l3 * cos_q3;
    k2 = l3 * sin_q3;
    cos_q2 = (k1 * r + k2 * (z2 - l1)) / (pow(k1,2) + pow(k2,2));
    sin_q2 = (k1 * (z2 -l1) - k2 * r) / (pow(k1,2) + pow(k2,2)); // 부호에 따라 elbow up & elbow down 결정 -> 현재 부호 = (+)
    q2 = atan2(sin_q2,cos_q2);
    q4 = phi - q2 - q3;
    ROS_INFO("q1 = %lf, q2 = %lf, q3 = %lf, q4 = %lf", q1, q2, q3, q4);
}

bool Callback(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    Request_Receive = 0;
    Response_Send = ros::Time::now().toSec();

    ROS_INFO("Client->Server : %lf time spend", Response_Send - Request_Receive);
    return true;
}

int main(int argc, char* argv[])
{
    // Init ROS node
    ros::init(argc, argv, "try_IK");
    ros::NodeHandle node_handle_;

    ros::Publisher angle_fromIK_pub = node_handle_.advertise<geometry_msgs::Twist>("/angle_fromIK", 10);
    ros::Subscriber pose_command_sub_ = node_handle_.subscribe("/goal_EE_position", 10, poseCallback);

    ros::ServiceServer service = node_handle_.advertiseService("/ping_tester", Callback);

    ros::Rate loop(250);

    ROS_INFO("try_IK!");

    geometry_msgs::Twist angle;

    while(ros::ok())
    {
        angle.linear.x = q1;
        angle.linear.y = q2;
        angle.linear.z = q3;
        angle.angular.x = q4;

        angle_fromIK_pub.publish(angle);

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}