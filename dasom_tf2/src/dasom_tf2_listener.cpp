#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"dasom_tf2_listener");
    ros::NodeHandle nh;

    // For /dasom/global_EE_frame/world
    tf2_ros::Buffer tfBuffer_globalEE;
    tf2_ros::TransformListener tfListener_globalEE(tfBuffer_globalEE);

    geometry_msgs::PoseStamped global_frame;
    geometry_msgs::Twist global_EE_meas_position;
    
    double roll, pitch, yaw;

    ros::Publisher global_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/tf/global_EE_pose", 10);
    ros::Publisher global_EE_meas_publisher = nh.advertise<geometry_msgs::Twist>("/dasom/tf/global_EE_meas_pose", 10);

    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped_globalEE;

        try
        {
            transformStamped_globalEE = tfBuffer_globalEE.lookupTransform("world", "tf/global_EE_pose", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // For /dasom/tf/global_EE_pose
        global_frame.pose.position.x = transformStamped_globalEE.transform.translation.x;
        global_frame.pose.position.y = transformStamped_globalEE.transform.translation.y;
        global_frame.pose.position.z = transformStamped_globalEE.transform.translation.z;

        global_frame.pose.orientation.x = transformStamped_globalEE.transform.rotation.x;
        global_frame.pose.orientation.y = transformStamped_globalEE.transform.rotation.y;
        global_frame.pose.orientation.z = transformStamped_globalEE.transform.rotation.z;
        global_frame.pose.orientation.w = transformStamped_globalEE.transform.rotation.w;

        global_publisher.publish(global_frame);

        global_EE_meas_position.linear = transformStamped_globalEE.transform.translation;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(global_frame.pose.orientation, quat);

        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        global_EE_meas_position.angular.x = roll;
        global_EE_meas_position.angular.y = pitch;
        global_EE_meas_position.angular.z = yaw;
        
        global_EE_meas_publisher.publish(global_EE_meas_position);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}