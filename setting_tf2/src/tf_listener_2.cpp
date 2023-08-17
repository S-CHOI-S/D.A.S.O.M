#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf/transform_datatypes.h"



int main(int argc, char** argv){
    ros::init(argc,argv,"tf_listener_2");

    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::PoseStamped gimbal_frame;
    


    ros::Publisher gimbal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/gimbal_EE_cmd", 10);

    ros::Rate rate(250);
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("paletrone", "global_gimbal_tf", ros::Time(0));
            }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }

         gimbal_frame.pose.position.x=transformStamped.transform.translation.x;
         gimbal_frame.pose.position.y=transformStamped.transform.translation.y;
         gimbal_frame.pose.position.z=transformStamped.transform.translation.z;

         gimbal_frame.pose.orientation.x=transformStamped.transform.rotation.x;
         gimbal_frame.pose.orientation.y=transformStamped.transform.rotation.y;
         gimbal_frame.pose.orientation.z=transformStamped.transform.rotation.z;
         gimbal_frame.pose.orientation.w=transformStamped.transform.rotation.w;


        gimbal_publisher.publish(gimbal_frame);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}