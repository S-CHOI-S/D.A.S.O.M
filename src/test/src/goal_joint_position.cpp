#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include "dynamixel_workbench_msgs/JointCommand.h"
#include "dynamixel_workbench_msgs/EECommand.h"
// #include "test/SetPosition.h"



/*void hapticCallback(const geometry_msgs::Twist &msg)
{
		joint1 = msg.linear.x;
		joint2 = msg.linear.y;
		joint3 = msg.linear.z;
		joint4 = msg.angular.x;
		joint5 = msg.angular.y;
		joint6 = msg.angular.z;
}
*/


/*
int main(int argc, char** argv)
{

	ros::init(argc, argv, "Goal_Joint_Position");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 100);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_tester_", 100);

//	ros::Subscriber hapticCallback_sub = n.subscribe("/now_haptic_endEffector_publisher", 10, hapticCallback);
 
	geometry_msgs::Twist cmd_;

	ros::Rate rate(250);


	double a = 0.1;

	int i = 0;

	while(ros::ok())
	{
	
	i++;

	if(i==500)
	{
		a = -a;
		i = 0;
	}

	//ROS_INFO("%lf", sine);


		sensor_msgs::JointState cmd_fromHaptic;
		cmd_fromHaptic.position.push_back(a);
		cmd_fromHaptic.position.push_back(a);
		cmd_fromHaptic.position.push_back(a);

		cmd_.linear.x = a;
		cmd_.linear.y = a;
		cmd_.linear.z = a;		
		publisher.publish(cmd_fromHaptic);
		pub.publish(cmd_);
		ros::spinOnce();
		rate.sleep();

	}

	return 0;


}


*/

double goal_position = 0;
double goal_position_2 = 0;

double amplitude = 0.02;
double sine_x = 0;
double period = 5;

double i = 0;

bool input(dynamixel_workbench_msgs::JointCommand::Request &req, dynamixel_workbench_msgs::JointCommand::Response &res)
{
	goal_position = req.goal_position;
	goal_position_2 = req.goal_position_2;

	ROS_INFO("joint1 = %lf", goal_position);
	ROS_INFO("joint2 = %lf", goal_position_2);
	return true;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "Goal_Joint_Position");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/joint_cmd",input);
	ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 100);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	// ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::EECommand>("/EE_command");

//	ros::Subscriber hapticCallback_sub = n.subscribe("/now_haptic_endEffector_publisher", 10, hapticCallback);
 
	geometry_msgs::Twist cmd_;
	dynamixel_workbench_msgs::EECommand srv;

	ros::Rate rate(250);


	// float sine = 0;
	// float i = 0;	


	// double Amplitude = 0;
	// double period = 0;

	
	
	// ROS_INFO("Amplitude : ");
	// std::cin >> Amplitude;

	// ROS_INFO("period : ");
	// std::cin >> period;

	// ROS_INFO("goal position[rad] : ");
	// std::cin >> goal_position;
	// std::cin >> goal_position_2;

	

	while(ros::ok())
	{
		// i = i + 0.004;

		// sine = Amplitude * cos(3.141592 * i / period);

		// //ROS_INFO("%lf", sine);


		// 	sensor_msgs::JointState cmd_fromHaptic;
		// 	cmd_fromHaptic.position.push_back(sine);
		// 	cmd_fromHaptic.position.push_back(sine + 0.05);
		// 	cmd_fromHaptic.position.push_back(sine - 0.05);

		// 	cmd_fromHaptic.velocity.push_back(sine + 0.05);
		// 	cmd_fromHaptic.velocity.push_back(sine - 0.05);

		// 	// cmd_.linear.x = sine;
		// 	// cmd_.linear.y = sine + 0.05;
		// 	// cmd_.linear.z = sine - 0.05;	

		// 	cmd_.linear.x = sine;
		// 	// cmd_.linear.y = sine + 0.05;
		// 	cmd_.angular.z = sine;

		// 	publisher.publish(cmd_fromHaptic);
		// 	pub.publish(cmd_);
		// 	ros::spinOnce();
		// 	rate.sleep();
	

	sine_x = amplitude*cos(3.141592*i/period) + 0.18;

	i += 0.004;

	// double x = 0.23209;
	// double y = 0;

	// x = sine_x;
	// y = 0;

	// srv.request.X = sine_x;
	// srv.request.Y = 0;

	// client.call(srv);
			
	sensor_msgs::JointState goal_;

	goal_.header.stamp = ros::Time::now();

	goal_.position.push_back(goal_position);
	goal_.position.push_back(goal_position_2);

	publisher.publish(goal_);

	ros::spinOnce();
	rate.sleep();
	}

	return 0;


}