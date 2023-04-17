
// x_a를 만들어서 x_c를 구하는 노드

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


double torque_constant = 4.4;

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


Eigen::Vector2d X_from_model_matrix;
Eigen::Vector2d X_dot_from_model_matrix;
Eigen::Vector2d Y_from_model_matrix;
Eigen::Vector2d Y_dot_from_model_matrix;

Eigen::Vector2d measured_effort;
Eigen::Vector2d measured_angle;
Eigen::Vector2d estimated_Force;
Eigen::Vector2d position_from_model;  //  MDK 모델에 의한 position
Eigen::Vector2d position_command;     //  최종 위치 
Eigen::Vector2d position_reference;   //  위치 입력값
Eigen::Vector2d command_position_fromGUI;


Eigen::Matrix2d Jacobian;
Eigen::Matrix2d Jacobian_transpose;

void commandcallback(const two_link::DasomDynamixel::ConstPtr &msg)
{
	command_position_fromGUI[0] = msg->position.at(0);
	command_position_fromGUI[1] = msg->position.at(1);
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
	ros::Subscriber CommandSub = n.subscribe("/dasom/reference_position", 10, commandcallback);  // Command From rqt
	ros::Subscriber JointSub = n.subscribe("/dasom/joint_states", 10, jointcallback); // Effort, Position, Velocity
	ros::Publisher Commandpub = n.advertise<two_link::DasomDynamixel>("/dasom/goal_position", 100); // Final Angle Command

	two_link::DasomDynamixel cmd;
//	ros::Subscriber hapticCallback_sub = n.subscribe("/now_haptic_endEffector_publisher", 10, hapticCallback);
 
//-----MDK Model ------//
	double m_x = 1.1;
	double b_x = 2.0;
	double k_x = 0.5;

//-----MDK Model ------//
	double m_y = 1.1;
	double b_y = 2.0;
	double k_y = 2.5;	


//----------Link Lengths---------//
	double Link1 = 0.1285;
	double Link2 = 0.108;

//-----State Space Representation----//
	A_x <<  0, 	1,
		  -k_x/m_x, -b_x/m_x;

	B_x << 0, 1/m_x;

	C_x << 1, 0;

	D_x << 0, 0;

//-----State Space Representation----//
	A_y <<  0, 	1,
		  -k_y/m_y, -b_y/m_y;

	B_y << 0, 1/m_y;

	C_y << 1, 0;

	D_y << 0, 0;



	X_from_model_matrix << 0, 0;
	X_dot_from_model_matrix << 0, 0;

	Y_from_model_matrix << 0, 0;
	Y_dot_from_model_matrix << 0, 0;

	time_i_loop = ros::Time::now().toSec(); // 키자마자 퍽 튀기 방지용

	ros::Rate rate(100);

	while(ros::ok())
	{



	two_link::DasomDynamixel cmd;

	time_f_loop = ros::Time::now().toSec();
	time_loop = time_f_loop - time_i_loop;
	time_i_loop = ros::Time::now().toSec();
	



	//포스 추정하자 
	Jacobian = solve_Jacobian(measured_angle[0], measured_angle[1], Link1, Link2);
	Jacobian_transpose = Jacobian.transpose();
	estimated_Force = Jacobian_transpose * measured_effort;




	//----------------------------------------------------------------
	// X 방향 admittance model 적용--------------------------------
	X_dot_from_model_matrix = A_x * X_from_model_matrix + B_x * estimated_Force[0];

	X_from_model_matrix = X_from_model_matrix + X_dot_from_model_matrix * time_loop;

	position_from_model[0] = X_from_model_matrix[0];

	position_command[0] = command_position_fromGUI[0] - position_from_model[0];


	//----------------------------------------------------------------
	// Y 방향 admittance model 적용--------------------------------
	Y_dot_from_model_matrix = A_y * Y_from_model_matrix + B_y * estimated_Force[1];

	Y_from_model_matrix = Y_from_model_matrix + Y_dot_from_model_matrix * time_loop;

	position_from_model[1] = Y_from_model_matrix[0];

	position_command[1] = command_position_fromGUI[1] - position_from_model[1];


//	ROS_INFO("%lf, %lf, %lf, %lf", X_from_model_matrix_T[0], X_from_model_matrix_T[1], X_dot_from_model_matrix_T[0], X_dot_from_model_matrix_T[1]);
//	ROS_INFO("position_from_model = %lf, position_reference = %lf, estimated_torque = %lf, time_loop = %lf", position_from_model, position_reference, estimated_torque, time_loop);
//	cmd.position.push_back(position_command[0]);
//	cmd.position.push_back(position_command[1]);
//	Commandpub.publish(cmd);
ROS_INFO("x_admit : %lf, y_admit : %lf", position_from_model[0], position_from_model[1]);


	ros::spinOnce();
	rate.sleep();

	}

	return 0;


}