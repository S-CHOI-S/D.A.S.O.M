#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>

double Subscribed_Time = 0;
void Test_SubCallback(const std_msgs::Float64 &str_msg)
{	
	Subscribed_Time = ros::Time::now().toSec();	
	ROS_INFO("Publisher -> Subscriber : %lf Times spend", Subscribed_Time -  str_msg.data);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Test_Sub");    

	ros::NodeHandle n;


	ros::Subscriber subscriber = n.subscribe("data", 1000, Test_SubCallback);  

	ros::spin();

	return 0;
}
