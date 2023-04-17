#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "test/dh.h"
#include "geometry_msgs/Twist.h"

using namespace std;

double q1;
double q2;
double q3;
double q4;

void Callback(const geometry_msgs::Twist &msg)
{
    q1 = msg.linear.x;
    q2 = msg.linear.y;
    q3 = msg.linear.z;
    q4 = msg.angular.x;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dh_FK");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/Test_Pub", 10, Callback);

    ros::Rate loop(250);

    double a2 = 1;
    double a3 = 1;
    double d3 = 0.2;
    double d4 = 0.2;

    while(ros::ok())
    {
        Eigen::Matrix4d T06 =
        DH(          0,  0,  0, q1) *  // T01
        DH(-EIGEN_PI/2,  0,  0, q2) *  // T12
        DH(          0, a2, d3, q3) *  // T23
        DH(-EIGEN_PI/2, a3, d4, q4);   // T34

        std::cout << T06.coeff(1,3) << T06.coeff(2,3) << T06.coeff(3,3) << std::endl << "-------------------" << std::endl;
        std::cout << 5.0 - T06.coeff(1,3) << 1 + T06.coeff(2,3) << 2 + T06.coeff(3,3) << std::endl << "-------------------" << std::endl;
                
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}
