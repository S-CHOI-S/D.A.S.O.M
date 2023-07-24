#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"tf_listener");

    ros::NodeHandle nh;

    ros::Publisher pos=nh.advertise<geometry_msgs::PoseStamped>("/joystick_cmd",100); //변환된 조이스틱 커맨드

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::Buffer tfBuffer2;
    tf2_ros::TransformListener tfListener2(tfBuffer2);

    geometry_msgs::PoseStamped xyzrpy;


    ros::Rate rate(250);
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
    	geometry_msgs::TransformStamped transformStamped2;
        try{
            transformStamped = tfBuffer.lookupTransform("joystickCMD", "world", ros::Time(0));
    	//    transformStamped2 = tfBuffer2.lookupTransform("joystickCMD","joystickBase",ros::Time(0));
            }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }

        xyzrpy.pose.position.x=transformStamped.transform.translation.x;
        xyzrpy.pose.position.y=transformStamped.transform.translation.y;
        xyzrpy.pose.position.z=transformStamped.transform.translation.z;

        xyzrpy.pose.orientation.x=transformStamped.transform.rotation.x;
        xyzrpy.pose.orientation.y=transformStamped.transform.rotation.y;
        xyzrpy.pose.orientation.z=transformStamped.transform.rotation.z;
        xyzrpy.pose.orientation.w=transformStamped.transform.rotation.w;

        pos.publish(xyzrpy);

        rate.sleep();
    }
    return 0;
}