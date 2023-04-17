#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <two_link/DasomDynamixel.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include "two_link/param.h"

int main(int argc, char **argv)
{
	// Init ROS node
	ros::init(argc, argv, "service_call");
	ros::NodeHandle node_handle_;
  	ros::ServiceClient client = node_handle_.serviceClient<two_link::param>("/set_params");

	if(argc != 3)
	{
		ROS_INFO("usage : service_call amplitude period");
		return 1;
	}

	two_link::param srv;

	srv.request.amplitude = atoll(argv[1]);
	srv.request.period = atoll(argv[2]);

	return 0;
}