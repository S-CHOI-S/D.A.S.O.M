
//이 노드는 xc를 받고 J도 계산하고 J-1도 계산하고 결국에 잘 버무려서 나온 최종 cmd_theta_dot을 dynamixel_workbench_controller::velocity_controller.cpp 로 쏘는 노드입니다/

// 초기값 : X = 0.20, Y = 0.05

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <two_link/DasomDynamixel.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include "two_link/param.h"
#include <dynamixel_workbench_msgs/DasomDynamixel.h>

Eigen::Vector2d P_gain;
Eigen::Vector2d I_gain;
Eigen::Vector2d D_gain;

Eigen::Vector2d command_position; // xc = (xr - xa)
Eigen::Vector2d position_dot; // (PID) * (position_des - position_measured)
Eigen::Vector2d angular_velocity_command; // 각 angle의 cmd_vel

//----------------position error --------------------//
Eigen::Vector2d error_i; //  포지션 에러 적분값
Eigen::Vector2d error_d; //  포지션 에러 미분값
Eigen::Vector2d error_p; //  포지션 에러
Eigen::Vector2d error_p_i; // error_d를 만들려고
Eigen::Vector2d position_dot_gain; // 포지션 에러에 PID 게인 곱한 결과

Eigen::Vector2d FK_position; // FK 로 구한 EndEffector 위치


Eigen::Vector2d measured_angle; // 각도 측정값
Eigen::Vector2d measured_position; // measured_angle에 FK를 씌움
Eigen::Matrix2d Jacobian;
Eigen::Matrix2d Jacobian_inverse;
Eigen::Matrix2d Jacobian_transpose;

double measured_x = 0;
double measured_y = 0;

double amplitude = 0;
double period = 0;

//-------------------루프 돌리기 용-------------------//
double time_f = 0;
double time_i = 0;
double time_loop = 0;


// void commandcallback(const two_link::DasomDynamixel::ConstPtr &msg)
// {
// 	command_position[0] = msg->position.at(0);
// 	command_position[1] = msg->position.at(1);
// }


void jointcallback(const dynamixel_workbench_msgs::DasomDynamixel &msg)
{
	measured_angle[0] = msg.position.at(0);
	measured_angle[1] = msg.position.at(1);
}


static Eigen::Matrix2d solve_Jacobian(double alpha, double beta, double Link1, double Link2)
{
	double cosA = cos(alpha), sinA = sin(alpha);
	double cosB = cos(beta), sinB = sin(beta);
	double cosAB = cos(alpha + beta), sinAB = sin(alpha + beta);
	Eigen::Matrix2d J;
	J <<  - Link1 * sinA - Link2 * sinAB,	-Link2 * sinAB,
		    Link1 * cosA + Link2 * cosAB,	 Link2 * cosAB;


	return J;
}


static Eigen::Matrix2d solve_Jacobian_Inverse(double alpha, double beta, double Link1, double Link2) /////////////////////////////////////
{
	double cosA = cos(alpha), sinA = sin(alpha);
	double cosB = cos(beta), sinB = sin(beta);
	double cosAB = cos(alpha + beta), sinAB = sin(alpha + beta);
	Eigen::Matrix2d J;
	J << 		 Link2 * cosAB,					   Link2 * sinAB,
		 -Link1 * cosA - Link2 * cosAB,		-Link1 * sinA - Link2 * sinAB;


	return J / (Link1 * Link2 * sinB);
}



static Eigen::Vector2d Forward_Kinematics(double alpha, double beta, double Link1, double Link2)
{
	Eigen::Vector2d FK;
	FK <<   Link1*cos(alpha) + Link2 * cos(alpha + beta),
			Link1*sin(alpha) + Link2 * sin(alpha + beta);
	return FK;
}


bool set_params(two_link::param::Request &req, two_link::param::Response &res)
{
	amplitude = req.amplitude;
	period = req.period;
	
	// ROS_INFO("amplitude = %lf", amplitude);
	// ROS_INFO("period = %lf \n", period);

	return true;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "velocity_tracking");
	ros::NodeHandle n;
	// ros::Subscriber CommandSub = n.subscribe("/dasom/reference_position", 10, commandcallback);  // Command From velocity_Admittance_Test.cpp
	ros::Publisher Commandpub = n.advertise<geometry_msgs::Twist>("/dasom/goal_dynamixel_position", 100); // Final Angle Command
	ros::Subscriber JointSub = n.subscribe("/dasom/joint_states", 10, jointcallback); // Effort, Position, Velocity
	ros::Publisher forPub = n.advertise<geometry_msgs::Twist>("/dasom/Test_Topic", 100); // 이것저것 쏘기 위함

	ros::ServiceServer service = n.advertiseService("/set_params",set_params);
	ROS_INFO("rosservice call /set_parmas");

//---------------------게인튜닝하자----------------//
	P_gain[0] = 400;
	I_gain[0] = 0;
	D_gain[0] = 10;

	P_gain[1] = 400;
	I_gain[1] = 0;
	D_gain[1] = 10;
//200 0 10
//400 0 10
	float sinu = 0;
	float i = 0;
	
	// ROS_INFO("amplitude : ");
	// std::cin >> amplitude;

	// ROS_INFO("period : ");
	// std::cin >> period;

	ros::Rate rate(100);

	geometry_msgs::Twist twist;
	geometry_msgs::Twist command;

	double Link1 = 0.10375;
	// double Link1 = 0.1285;
	double Link2 = 0.108;


	time_i = ros::Time::now().toSec();

	while(ros::ok())
	{
		ROS_INFO("amplitude = %lf", amplitude);
		ROS_INFO("period = %lf \n", period);

		// sinusoidal input
		i = i + 0.004;

		// sinu = 0.16 + 0.04 * cos(3.141592 * i / 2);
		// sinu = 0.21 + 0.02 * cos(3.141592 * i / 2);
		sinu = 10 * sin(3.141592 * i / 2);

		position_dot << sinu, 0;

		ROS_WARN("sinu = %lf", sinu);

		// command_position[0] = sinu;
		// command_position[1] = 0.05;

		// command_position[1] = // 상수 C -> why? 0을 적분하면 적분 상수 C가 만들어짐!


	//------------dt 구하기 용------------//
		time_f = ros::Time::now().toSec();
		time_loop = time_f - time_i;
		time_i = ros::Time::now().toSec();
		ROS_INFO("angle1 = %lf",measured_angle[0]);
		ROS_INFO("angle2 = %lf",measured_angle[1]);
		ROS_ERROR("time loop = %lf", time_loop);
		Jacobian = solve_Jacobian(measured_angle[0], measured_angle[1], Link1, Link2);
		Jacobian_transpose = Jacobian.transpose();
//		Jacobian_inverse = solve_Jacobian_Inverse(measured_angle[0], measured_angle[1], Link1, Link2);

		measured_position = Forward_Kinematics(measured_angle[0], measured_angle[1], Link1, Link2);

		// command position 값과 FK 구한 값 비교
		// command.linear.x = command_position[0];
		// command.linear.y = command_position[1];
		command.angular.x = measured_position[0];
		command.angular.y = measured_position[1];

		// ROS_ERROR("X_pos_cmd = %lf", command_position[0]);
		ROS_ERROR("X_pos_FKK = %lf", measured_position[0]);
		// ROS_ERROR("Y_pos_cmd = %lf", command_position[1]);
		ROS_ERROR("Y_pos_FKK = %lf", measured_position[1]);


	//------------------------------------------------//
 

	//------------------------------------------------//


	//---------------포지션 에러 구하기 ---------------//


		// for(int i = 0; i<2; i++)
		// {
		// 	error_p[i] = command_position[i] - measured_position[i]; // error_p = error
		// 	error_i[i] = error_i[i] + error_p[i] * time_loop;
		// 	if (error_p[i] - error_p_i[i] != 0) error_d[i] = (error_p[i] - error_p_i[i]) / time_loop;
		// 	error_p_i[i] = error_p[i];
			
		// 	position_dot_gain[i] = P_gain[i] * error_p[i] + I_gain[i] * error_i[i] + D_gain[i] * error_d[i];
		// }

		// ROS_INFO("position_dot_gain0 = %lf", position_dot_gain[0]);
		// ROS_INFO("position_dot_gain1 = %lf", position_dot_gain[1]);

		angular_velocity_command = Jacobian_transpose * position_dot;

		ROS_INFO("ang1_vel_cmd = %lf", angular_velocity_command[0]);
		ROS_INFO("ang2_vel_cmd = %lf", angular_velocity_command[1]);

		twist.linear.x = angular_velocity_command[0];
		twist.angular.z = angular_velocity_command[1];
		
		Commandpub.publish(twist);


		forPub.publish(command);

		ros::spinOnce();
		rate.sleep();


	// std::cout << Jacobian_inverse << "\n ---------------- \n";
	// std::cout << Jacobian_inverse_ << "\n =================== \n";
	
	// ROS_INFO("[cmd_x] : %lf, [cmd_y] : %lf ", command_position[0], command_position[1]);
	// ROS_INFO("[FK_x] : %lf, [FK_y] : %lf ", measured_position[0], measured_position[1]);
	// ROS_INFO("[vel_x] : %lf, [vel_y] : %lf", angular_velocity_command[0], angular_velocity_command[1]);
	// ROS_INFO("[ang1] : %lf, [ang2] : %lf ", measured_angle[0], measured_angle[1]);

	

	//	ROS_INFO("%lf, %lf \n", angular_velocity_command[0], angular_velocity_command[1]);
	//	ROS_INFO("x error : %lf, y error : %lf", command_position[0] - measured_position[0], command_position[1] - measured_position[1]);


	}
	return 0;



}


// velocity 값 잘 따라가고 있는 지 확인?
// 1. /dasom/goal_dynamixel_position/linear/x : /dasom/joint_states/velocity[0]
// 2. /dasom/goal_dynamixel_position/angular/z : /dasom/joint_states/velocity[1]

// FK 잘 따라가고 있는 지 확인?
// 1. /dasom/Test_Topic/linear/x : /dasom/Test_Topic/angular/x
// 2. /dasom/Test_Topic/linear/y : /dasom/Test_Topic/angular/y