#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf/transform_datatypes.h"

//전달사항
//lookuptransform 고침
//근데 global_EE_pose_tf 이게 값이 말도 안되게 이상하게나옴
// 그래서그런지, 버튼 누ㅡㄹ면 out of workspce나옴
//일단 global ee pose tf 이거부터 고쳐야할듯




int main(int argc, char** argv){
    ros::init(argc,argv,"tf_listener");

    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::PoseStamped gimbal_cmd;
    geometry_msgs::PoseStamped gimbal_frame;
    
    geometry_msgs::Twist check;


    ros::Publisher gimbal_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/gimbal_frame", 10);
    ros::Publisher gimbal_cmd_publisher = nh.advertise<geometry_msgs::PoseStamped>("/dasom/gimbal_tf", 10);



    ros::Rate rate(250);
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("world", "global_EE_pose_tf", ros::Time(0));
            }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }

         gimbal_cmd.pose.position.x=transformStamped.transform.translation.x;
         gimbal_cmd.pose.position.y=transformStamped.transform.translation.y;
         gimbal_cmd.pose.position.z=transformStamped.transform.translation.z;

         gimbal_cmd.pose.orientation.x=transformStamped.transform.rotation.x;
         gimbal_cmd.pose.orientation.y=transformStamped.transform.rotation.y;
         gimbal_cmd.pose.orientation.z=transformStamped.transform.rotation.z;
         gimbal_cmd.pose.orientation.w=transformStamped.transform.rotation.w;




  tf::Quaternion quat;
  tf::quaternionMsgToTF(gimbal_cmd.pose.orientation,quat);

  tf::Matrix3x3(quat).getRPY(check.angular.x, check.angular.y, check.angular.z);

    // ROS_INFO("AAAAA %lf, %lf, %lf, %lf, %lf, %lf",gimbal_cmd.pose.position.x, gimbal_cmd.pose.position.y, gimbal_cmd.pose.position.z, check.angular.x, check.angular.y, check.angular.z);



        gimbal_cmd_publisher.publish(gimbal_cmd);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}