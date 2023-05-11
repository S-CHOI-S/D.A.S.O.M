/*******************************************************************************
* D.A.S.O.M
*
* Department of Aerial Manipulator System for Object Manipulation
*
*     https://github.com/S-CHOI-S/D.A.S.O.M.git
*
* Mobile Robotics Lab. (MRL)
* 	  @ Seoul National University of Science and Technology
*
*	  https://mrl.seoultech.ac.kr/index.do
*
*******************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "dynamixel_workbench_msgs/EECommand.h"

double X_command = 0.256;
double Y_command = 0;

bool EndEffectorCommand(dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res)
// rosservice call /EE_command X Y
{
	X_command = req.X;
	Y_command = req.Y;

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_joint_position");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("/goal_EE_position", 100);
	ros::ServiceServer service = n.advertiseService("/EE_command", EndEffectorCommand);

	ROS_INFO("[goal_joint_position] node start!");
	// dynamixel_workbench_msgs::EECommand srv;

	ros::Rate rate(250);	

	while(ros::ok())
	{			
		sensor_msgs::JointState goal_EE;

		goal_EE.header.stamp = ros::Time::now();

		goal_EE.position.push_back(X_command);
		goal_EE.position.push_back(Y_command);

		publisher.publish(goal_EE);

		ROS_INFO("%lf, %lf",X_command,Y_command);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;


}