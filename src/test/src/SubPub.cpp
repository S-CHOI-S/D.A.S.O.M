#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>




class TestClass
{
public:
	TestClass()
	{
	subscriber = nh.subscribe("PubSub", 1000, &TestClass::TestCallback, this);
	publisher = nh.advertise<std_msgs::Float64>("SubPub", 1000);
	}

void TestCallback(const std_msgs::Float64 &str_msg)    //받자마자 보내기
	{
	publisher.publish(SubPub);
	ROS_INFO("receive and send");
	}
private:
	ros::NodeHandle nh;
	ros::Publisher publisher;
	ros::Subscriber subscriber;
	std_msgs::Float64 SubPub;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "TestClass");

	TestClass test;
	ros::spin();

	return 0;

}
