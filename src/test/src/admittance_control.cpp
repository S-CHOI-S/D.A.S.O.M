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

Eigen::Vector2d X_i;
Eigen::Vector2d X_f;
Eigen::Vector2d Xdot;

Eigen::Vector2d X_i_T;
Eigen::Vector2d X_f_T;
Eigen::Vector2d Xdot_T;


void commandcallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg)
{
	command_position_fromGUI = msg->position.at(0);
	position_reference = command_position_fromGUI;
	// ROS_INFO("position_reference = %lf", position_reference); // 잘 나오고 있음
}

void jointcallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg)
// /dasom/joint_states에서 position, velocity, effort 값을 받아와서 [토크 계산(estimated_torque)], [가속도 수치 미분(estimated_ang_vel)]
{
	time_f_callback = ros::Time::now().toSec();
	time_callback = time_f_callback = time_i_callback;

	measured_angle = msg->position.at(0);
	measured_velocity_f = msg->velocity.at(0);
	measured_effort = msg->effort.at(0);
	estimated_torque = measured_effort * torque_constant;

	// 가속도 수치 미분
	if(measured_velocity_f - measured_velocity_i != 0) estimated_ang_vel = (measured_velocity_f - measured_velocity_i) / (time_f_callback - time_i_callback);
	
	measured_velocity_i = measured_velocity_f;

	// ROS_INFO("%lf", measured_velocity_f);
	// ROS_INFO("%lf", estimated_torque);

	time_i_callback = ros::Time::now().toSec();
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
	double m = 0.01;
	double b = 0.01;
	double k = 0.01;

//-----State Space Representation----//
	A <<  0, 	1,
		-k/m, -b/m;

	B << 0, 1/m;

	C << 1, 0;

	D << 0, 0;

	BT = B.transpose();

	X_f << 0, 0; // [x x']
	X_i << 0, 0; // [x x']
	Xdot << 0, 0; // [x' x'']

	X_f_T = X_f.transpose();
	X_i_T = X_i.transpose();
	Xdot_T = Xdot.transpose();

	time_i_loop = ros::Time::now().toSec(); // 키자마자 퍽 튀기 방지용

	ros::Rate rate(250);



	while(ros::ok())
	{


	dynamixel_workbench_msgs::DasomDynamixel cmd;

	time_f_loop = ros::Time::now().toSec();
	time_loop = time_f_loop - time_i_loop;

	// position_from_model = 2*k/(m*sqrt(b*b-4*k*m))*sinh(sqrt(b*b-4*k*m)/(2*m)*time_loop)*exp(-b/(2*m)*time_loop) * estimated_torque;
	// position_from_model = 1/(sqrt(k/m-pow(b,2)/(4*pow(m,2))))*sin(sqrt(k/m-pow(b,2)/(4*pow(m,2)))*time_loop)*exp(-b/(2*m)*time_loop) * estimated_torque;

	// position_command = position_reference + position_from_model;   //이건 그냥 전달함수 S domain -> T domain 으로 바꿔서 넣은 거
	// position_command = position_reference - position_from_model;

	Xdot_T = A * X_f_T + B * F;
	// // ROS_INFO("%lf,%lf",Xdot_T[0],Xdot_T[1]);

	X_f_T = X_f_T + Xdot_T * time_loop;

	// X_f_T[0] = position_command; // ?
	position_command = X_f_T[0];

	//	ROS_INFO("pc=%lf, pr=%lf, pm=%lf, torque=%lf, time=%lf", position_command, position_reference, position_from_model, estimated_torque, time);
	// ROS_INFO("position_command = %lf, time_f_callback = %lf", X_f_T[0], X_f_T[1]);

	cmd.position.push_back(position_command);
	Commandpub.publish(cmd);



	time_i_loop = ros::Time::now().toSec();
	ros::spinOnce();
	rate.sleep();

	}

	return 0;


}