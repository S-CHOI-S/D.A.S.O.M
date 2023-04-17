#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 


double command_position_fromGUI = 0;
double command_position_toDynxel = 0;

double position_from_model = 0;  //  MDK 모델에 의한 position
double position_dot_from_model = 0;
double position_ddot_from_model = 0;

double position_reference = 0;   //  위치 입력값
double position_command = 0;     //  최종 위치 

double measured_velocity_i = 0;
double measured_velocity_f = 0;
double measured_angle = 0;
double measured_effort = 0;
double estimated_torque = 0;
double torque_constant = 1;

double estimated_ang_vel = 0;

double time_i_callback = 0;
double time_f_callback = 0;
double time_callback = 0;

double time_i_loop = 0;
double time_f_loop = 0;
double time_loop = 0;
double F = 0;


Eigen::Matrix2d A;
Eigen::Vector2d B;
Eigen::Vector2d C;
Eigen::Vector2d D;
Eigen::Vector2d BT; //B transpose
Eigen::Vector2d DT; //B transpose


Eigen::Vector2d X_from_model_matrix;
Eigen::Vector2d X_dot_from_model_matrix;
Eigen::Vector2d X_from_model_matrix_T;
Eigen::Vector2d X_dot_from_model_matrix_T;



void commandcallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg)
{
	command_position_fromGUI = msg->position.at(0);
	position_reference = command_position_fromGUI;
}

void jointcallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg)
{
	time_f_callback = ros::Time::now().toSec();
	time_callback = time_f_callback = time_i_callback;
	time_i_callback = ros::Time::now().toSec();


	measured_angle = msg->position.at(0);
	measured_velocity_f = msg->velocity.at(0);
	measured_effort = msg->effort.at(0);
	estimated_torque = measured_effort * torque_constant;

	if(measured_velocity_f - measured_velocity_i != 0) estimated_ang_vel = (measured_velocity_f - measured_velocity_i) / (time_f_callback - time_i_callback);
	
	measured_velocity_i = measured_velocity_f;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "Admittance_Test");
	ros::NodeHandle n;
	ros::Subscriber CommandSub = n.subscribe("/dasom/goal_position", 10, commandcallback);  // Command From rqt
	ros::Subscriber JointSub = n.subscribe("/dasom/joint_states", 10, jointcallback); // Effort, Position, Velocity
	ros::Publisher Commandpub = n.advertise<dynamixel_workbench_msgs::DasomDynamixel>("/dasom/goal_dynamixel_position", 100); // Final Angle Command

	dynamixel_workbench_msgs::DasomDynamixel cmd;
//	ros::Subscriber hapticCallback_sub = n.subscribe("/now_haptic_endEffector_publisher", 10, hapticCallback);
 
//-----MDK Model ------//
	double m = 1;
	double b = 1;
	double k = 1;

//-----State Space Representation----//
	A <<  0, 	1,
		-k/m, -b/m;

	B << 0, 1/m;

	C << 1, 0;

	D << 0, 0;

	BT = B.transpose();

	X_from_model_matrix << 0, 0;
	X_dot_from_model_matrix << 0, 0;

	X_from_model_matrix_T = X_from_model_matrix.transpose();
	X_dot_from_model_matrix_T = X_dot_from_model_matrix.transpose();

	time_i_loop = ros::Time::now().toSec(); // 키자마자 퍽 튀기 방지용

	ros::Rate rate(250);



	while(ros::ok())
	{



	dynamixel_workbench_msgs::DasomDynamixel cmd;

	time_f_loop = ros::Time::now().toSec();
	time_loop = time_f_loop - time_i_loop;
	time_i_loop = ros::Time::now().toSec();
	





	X_dot_from_model_matrix_T = A * X_from_model_matrix_T + BT * estimated_torque;

	X_from_model_matrix_T = X_from_model_matrix_T + X_dot_from_model_matrix_T * time_loop;

	position_from_model = X_from_model_matrix_T[0];

	position_command = position_reference - position_from_model;

	ROS_INFO("%lf, %lf, %lf, %lf", X_from_model_matrix_T[0], X_from_model_matrix_T[1], X_dot_from_model_matrix_T[0], X_dot_from_model_matrix_T[1]);
//	ROS_INFO("position_from_model = %lf, position_reference = %lf, estimated_torque = %lf, time_loop = %lf", position_from_model, position_reference, estimated_torque, time_loop);
	cmd.position.push_back(position_command);
	Commandpub.publish(cmd);



	ros::spinOnce();
	rate.sleep();

	}

	return 0;


}