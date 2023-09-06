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
    ros::init(argc,argv,"tf_listener");
    ros::NodeHandle nh;

    // For /dasom/global_EE_frame/world
    tf2_ros::Buffer tfBuffer_globalEE;
    tf2_ros::TransformListener tfListener_globalEE(tfBuffer_globalEE);

    // For /dasom/gimbal_EE_cmd
    tf2_ros::Buffer tfBuffer_gimbalEE;
    tf2_ros::TransformListener tfListener_gimbalEE(tfBuffer_gimbalEE);

    geometry_msgs::PoseStamped global_frame;
    geometry_msgs::PoseStamped gimbal_frame;
    
    ros::Publisher global_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/global_EE_frame/world", 10);
    ros::Publisher gimbal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/gimbal_EE_cmd", 10);

    ros::Rate rate(250);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped_globalEE;
        geometry_msgs::TransformStamped transformStamped_gimbalEE;

        try
        {
            transformStamped_globalEE = tfBuffer_globalEE.lookupTransform("world", "global_EE_pose", ros::Time(0));
            transformStamped_gimbalEE = tfBuffer_gimbalEE.lookupTransform("paletrone", "global_gimbal_tf", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // For /dasom/global_EE_frame/world
        global_frame.pose.position.x = transformStamped_globalEE.transform.translation.x;
        global_frame.pose.position.y = transformStamped_globalEE.transform.translation.y;
        global_frame.pose.position.z = transformStamped_globalEE.transform.translation.z;

        global_frame.pose.orientation.x = transformStamped_globalEE.transform.rotation.x;
        global_frame.pose.orientation.y = transformStamped_globalEE.transform.rotation.y;
        global_frame.pose.orientation.z = transformStamped_globalEE.transform.rotation.z;
        global_frame.pose.orientation.w = transformStamped_globalEE.transform.rotation.w;

        // For /dasom/gimbal_EE_cmd
        gimbal_frame.pose.position.x = transformStamped_gimbalEE.transform.translation.x;
        gimbal_frame.pose.position.y = transformStamped_gimbalEE.transform.translation.y;
        gimbal_frame.pose.position.z = transformStamped_gimbalEE.transform.translation.z;

        gimbal_frame.pose.orientation.x = transformStamped_gimbalEE.transform.rotation.x;
        gimbal_frame.pose.orientation.y = transformStamped_gimbalEE.transform.rotation.y;
        gimbal_frame.pose.orientation.z = transformStamped_gimbalEE.transform.rotation.z;
        gimbal_frame.pose.orientation.w = transformStamped_gimbalEE.transform.rotation.w;

        global_publisher.publish(global_frame);
        gimbal_publisher.publish(gimbal_frame);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}