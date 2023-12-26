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
    ros::init(argc,argv,"dasom_drone_tf_listener2");
    ros::NodeHandle nh;

    // For /dasom/gimbal_EE_cmd
    tf2_ros::Buffer tfBuffer_gimbalEE;
    tf2_ros::TransformListener tfListener_gimbalEE(tfBuffer_gimbalEE);

    geometry_msgs::PoseStamped gimbal_frame;
    
    ros::Publisher gimbal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/tf/global_gimbal_command", 10);

    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped_gimbalEE;

        try
        {
            transformStamped_gimbalEE = tfBuffer_gimbalEE.lookupTransform("tf/paletrone_drone_converter_measured", "tf/converted_global_fixed_gimbal_tf", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // For /dasom/tf/global_gimbal_command
        gimbal_frame.pose.position.x = transformStamped_gimbalEE.transform.translation.x;
        gimbal_frame.pose.position.y = transformStamped_gimbalEE.transform.translation.y;
        gimbal_frame.pose.position.z = transformStamped_gimbalEE.transform.translation.z;

        gimbal_frame.pose.orientation.x = transformStamped_gimbalEE.transform.rotation.x;
        gimbal_frame.pose.orientation.y = transformStamped_gimbalEE.transform.rotation.y;
        gimbal_frame.pose.orientation.z = transformStamped_gimbalEE.transform.rotation.z;
        gimbal_frame.pose.orientation.w = transformStamped_gimbalEE.transform.rotation.w;

        gimbal_publisher.publish(gimbal_frame);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}