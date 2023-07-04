#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <two_link/DasomDynamixel.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 


double command_position_toDynxel = 0;

double position_dot_from_model = 0;
double position_ddot_from_model = 0;

//------------보정치--------------//
double torque_constant_1 = 0.174;
double torque_constant_2 = 0.174;

double offset_1 = 0.35;
double offset_2 = 0.25;
//-------------------------------


double estimated_ang_vel = 0;

double time_i_callback = 0;
double time_f_callback = 0;
double time_callback = 0;

double time_i_loop = 0;
double time_f_loop = 0;
double time_loop = 0;
double F = 0;


Eigen::Matrix2d A_x;
Eigen::Vector2d B_x;
Eigen::Vector2d C_x;
Eigen::Vector2d D_x;

Eigen::Matrix2d A_y;
Eigen::Vector2d B_y;
Eigen::Vector2d C_y;
Eigen::Vector2d D_y;

Eigen::Vector2d BT; //B transpose
Eigen::Vector2d DT; //B transpose


Eigen::Vector2d virtual_mass;
Eigen::Vector2d virtual_damper;
Eigen::Vector2d virtual_spring;

Eigen::Vector2d X_from_model_matrix;
Eigen::Vector2d X_dot_from_model_matrix;
Eigen::Vector2d Y_from_model_matrix;
Eigen::Vector2d Y_dot_from_model_matrix;

Eigen::Vector2d measured_effort;
Eigen::Vector2d measured_torque;
Eigen::Vector2d measured_angle;
Eigen::Vector2d estimated_Force;
Eigen::Vector2d position_from_model;  //  MDK 모델에 의한 position
Eigen::Vector2d position_command;     //  최종 위치 
Eigen::Vector2d position_reference;   //  위치 입력값
Eigen::Vector2d Tau_gravity; //중력에 의해 조인트에 가해지는 토크
Eigen::Vector2d Tau_external_force; // 외력에 의해 조인트에 가해지는 토크 (중력성분을 빼서 구했음)


Eigen::Matrix2d Jacobian;
Eigen::Matrix2d Jacobian_transpose;

void commandcallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	position_reference[0] = msg->position.at(0);
	position_reference[1] = msg->position.at(1);
}

void jointcallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	time_f_callback = ros::Time::now().toSec();
	time_callback = time_f_callback = time_i_callback;
	time_i_callback = ros::Time::now().toSec();


	measured_angle[0] = msg->position.at(0);
	measured_angle[1] = msg->position.at(1);
	measured_effort[0] = msg->effort.at(0);
	measured_effort[1] = msg->effort.at(1);

	measured_torque[0] = measured_effort[0] * torque_constant_1;
	measured_torque[1] = measured_effort[1] * torque_constant_2;
}

static Eigen::Matrix2d solve_Jacobian(double alpha, double beta, double Link1, double Link2)
{
	double cosA = cos(alpha), sinA = sin(alpha);
	double cosB = cos(beta), sinB = sin(beta);
	double cosAB = cos(alpha + beta), sinAB = sin(alpha + beta);
	Eigen::Matrix2d J;
	J << - Link1 * sinA - Link2 * sinAB,	-Link2 * sinAB,
		   Link1 * cosA + Link2 * cosAB,	 Link2 * cosAB;

	return J;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "Admittance_twoLink");
	ros::NodeHandle n;
	ros::Subscriber CommandSub = n.subscribe("/reference_position", 10, commandcallback);  // Command From rqt
	ros::Subscriber JointSub = n.subscribe("/joint_states", 10, jointcallback); // Effort, Position, Velocity
	ros::Publisher Commandpub = n.advertise<sensor_msgs::JointState>("/goal_EE_position", 100); // Final Angle Command
	ros::Publisher testPub = n.advertise<geometry_msgs::Twist>("/ForceTest", 100);

	two_link::DasomDynamixel cmd;
//	ros::Subscriber hapticCallback_sub = n.subscribe("/now_haptic_endEffector_publisher", 10, hapticCallback);
 
//-----MDK Model ------//
	virtual_mass[0] = 1.1;
	virtual_damper[0] = 2.0;
	virtual_spring[0] = 0.5;

//-----MDK Model ------//
	virtual_mass[1] = 1.1;
	virtual_damper[1] = 2.0;
	virtual_spring[1] = 0.5;


//----------Link Lengths---------//
	double Link1 = 0.10409;
	double Link2 = 0.13377;

//----------Link Lengths---------//
	double CoM1 = 0.08092;
	double CoM2 = 0.114;
	double delta = 0.237;
	double mass1 = 0.107;
	double mass2 = 0.296;

//-----State Space Representation----//
	A_x <<                0, 									1,
		  - virtual_spring[0]/virtual_mass[0], - virtual_damper[0]/virtual_mass[0];

	B_x << 0, 1/virtual_mass[0];

	C_x << 1, 0;

	D_x << 0, 0;

//-----State Space Representation----//
	A_y <<               0, 									1,
		  - virtual_spring[1]/virtual_mass[1], - virtual_damper[1]/virtual_mass[1];

	B_y << 0, 1/virtual_mass[1];

	C_y << 1, 0;

	D_y << 0, 0;



	X_from_model_matrix << 0, 0;
	X_dot_from_model_matrix << 0, 0;

	Y_from_model_matrix << 0, 0;
	Y_dot_from_model_matrix << 0, 0;

	time_i_loop = ros::Time::now().toSec(); // 키자마자 퍽 튀기 방지용

	ros::Rate rate(250);

	geometry_msgs::Twist ForceTest;
	while(ros::ok())
	{



	sensor_msgs::JointState cmd;

	time_f_loop = ros::Time::now().toSec();
	time_loop = time_f_loop - time_i_loop;
	time_i_loop = ros::Time::now().toSec();
	

	//----------------------------------------------------------------
	// 중력 매트릭스 (OCM 있을 때)--------------------------------
	Tau_gravity << (mass1 * CoM1 * cos(measured_angle[0]) + mass2 * Link1 * cos(measured_angle[0]) + mass2 * CoM2 * cos(measured_angle[0] + measured_angle[1] + delta)) * 9.81 - offset_1,
					mass2 * CoM2 * cos(measured_angle[0] + measured_angle[1] + delta) * 9.81 - offset_2;

	Tau_external_force = measured_torque - Tau_gravity;
	//----------------------------------------------------------------
	//End Effector 외력 추정 
	Jacobian = solve_Jacobian(measured_angle[0], measured_angle[1], Link1, Link2);
	Jacobian_transpose = Jacobian.transpose();
	estimated_Force = Jacobian_transpose * Tau_external_force;


	//----------------------------------------------------------------
	// X 방향 admittance model 적용--------------------------------
	X_dot_from_model_matrix = A_x * X_from_model_matrix + B_x * estimated_Force[0];

	X_from_model_matrix = X_from_model_matrix + X_dot_from_model_matrix * time_loop;

	position_from_model[0] = X_from_model_matrix[0];

	//position_command[0] = position_reference[0] - position_from_model[0]; 아직 무서우니까~
	position_command[0] = position_reference[0];

	//----------------------------------------------------------------
	// Y 방향 admittance model 적용--------------------------------
	Y_dot_from_model_matrix = A_y * Y_from_model_matrix + B_y * estimated_Force[1];

	Y_from_model_matrix = Y_from_model_matrix + Y_dot_from_model_matrix * time_loop;

	position_from_model[1] = Y_from_model_matrix[0];

	//position_command[1] = position_reference[1] - position_from_model[1]; 아직 무서우니까~
	position_command[1] = position_reference[1];


	//----------------------------------------------------------------
	// 잘 되나 확인을 해보자--------------------------------

	ROS_INFO("Gravity matrix : [%lf], [%lf]", Tau_gravity[0], Tau_gravity[1]);
	ROS_INFO("torque : [%lf], [%lf]", measured_torque[0], measured_torque[1]);
	ROS_INFO("G - Tau : [%lf], [%lf]", Tau_gravity[0] - measured_torque[0], Tau_gravity[1] - measured_torque[1]);
	ROS_INFO("x_admittance : [%lf], [%lf]", position_from_model[0], position_from_model[1]);
	ROS_INFO("position_command : [%lf], [%lf]", position_command[0], position_command[1]);
	ROS_INFO("==================\n");



	//----------------------------------------------------------------
	// 이것저것 테스트 plot 띄우기용 topic --------------------------------

	ForceTest.linear.x = Tau_gravity[0];
	ForceTest.linear.y = Tau_gravity[1];
	ForceTest.angular.x = measured_torque[0];
	ForceTest.angular.y = measured_torque[1];
	ForceTest.linear.z = estimated_Force[0];
	ForceTest.angular.z = estimated_Force[1];
	

	testPub.publish(ForceTest); 
		
	
	//----------------------------------------------------------------
	// 움직여라 명령! --------------------------------

	cmd.position.push_back(position_command[0]);
	cmd.position.push_back(position_command[1]);
	Commandpub.publish(cmd);


	ros::spinOnce();
	rate.sleep();

	}

	return 0;


}