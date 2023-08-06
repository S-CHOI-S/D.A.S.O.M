#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <dasom_toolbox/dasom_camera.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "try");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
    ////////////////////
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/dasom/camera_image", 1);
    //////////////////// 

    ros::Rate loop_rate(20);

    double joint1 = 0, joint2 = 0, joint3 = 0, joint4 = 0;
    double i = 0;
    double t = 0;

    sensor_msgs::JointState joint_states;
    ////////////////////
    DasomCam ds_cam_(pub, 0);
    ////////////////////
    while (ros::ok())
    {ds_cam_.UpdateCamera(); ////////////////////
        //update joint_state
        joint_states.header.stamp = ros::Time::now();
        joint_states.name.resize(4);
        joint_states.position.resize(4);
        joint_states.name[0] = "joint1";
        joint_states.position[0] = joint1;
        joint_states.name[1] = "joint2";
        joint_states.position[1] = joint2;
        joint_states.name[2] = "joint3";
        joint_states.position[2] = joint3;
        joint_states.name[3] = "joint4";
        joint_states.position[3] = joint4;

        t = i / 100;
        joint1 = t;
        joint2 = 1 + t;
        joint3 = -t;
        joint4 = t;

        joint_pub.publish(joint_states);

        i++;

        ROS_ERROR("i: %lf", i);
        if(i > 157) break;

        ROS_INFO("joint1: %lf", joint1);
        ROS_INFO("joint2: %lf", joint2);
        ROS_INFO("joint3: %lf", joint3);
        ROS_INFO("joint4: %lf", joint4);
        ROS_WARN("==========================");

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}