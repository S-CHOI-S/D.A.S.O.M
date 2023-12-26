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
    ros::init(argc,argv,"dasom_drone_tf_listener3");
    ros::NodeHandle nh;

    // For /dasom/gimbal_EE_cmd
    tf2_ros::Buffer tfBuffer_gimbalEE;
    tf2_ros::TransformListener tfListener_gimbalEE(tfBuffer_gimbalEE);

    geometry_msgs::Twist gimbal_frame;

    double roll, pitch, yaw;
    
    ros::Publisher gimbal_publisher = nh.advertise<geometry_msgs::Twist>("/dasom/tf/global_EE_cmd", 10);

    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped_gimbalEE;

        try
        {
            transformStamped_gimbalEE = tfBuffer_gimbalEE.lookupTransform("world", "tf/converted_global_EE_cmd_pose", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // For /dasom/tf/global_EE_cmd
        gimbal_frame.linear.x = transformStamped_gimbalEE.transform.translation.x;
        gimbal_frame.linear.y = transformStamped_gimbalEE.transform.translation.y;
        gimbal_frame.linear.z = transformStamped_gimbalEE.transform.translation.z;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(transformStamped_gimbalEE.transform.rotation, quat);

        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        gimbal_frame.angular.x = roll;
        gimbal_frame.angular.y = pitch;
        gimbal_frame.angular.z = yaw;

        gimbal_publisher.publish(gimbal_frame);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}