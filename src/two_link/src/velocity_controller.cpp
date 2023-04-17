
//이 노드는 xc를 받고 J도 계산하고 J-1도 계산하고 결국에 잘 버무려서 나온 최종 cmd_theta_dot을 dynamixel_workbench_controller::velocity_controller.cpp 로 쏘는 노드입니다/



#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <two_link/DasomDynamixel.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 

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
Eigen::Vector2d position_gain; // 포지션 에러에 PID 게인 곱한 결과

Eigen::Vector2d FK_position; // FK 로 구한 EndEffector 위치


Eigen::Vector2d measured_angle; // 각도 측정값
Eigen::Vector2d measured_position; // measured_angle에 FK를 씌움
Eigen::Matrix2d Jacobian;
Eigen::Matrix2d Jacobian_inverse;
Eigen::Matrix2d Jacobian_inverse_;

double measured_x = 0;
double measured_y = 0;


//-------------------루프 돌리기 용-------------------//
double time_f = 0;
double time_i = 0;
double time_loop = 0;


void commandcallback(const two_link::DasomDynamixel::ConstPtr &msg)
{
	command_position[0] = msg->position.at(0);
	command_position[1] = msg->position.at(1);
}


void jointcallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	measured_angle[0] = msg->position.at(0);
	measured_angle[1] = msg->position.at(1);
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





int main(int argc, char** argv)
{
	ros::init(argc, argv, "Admittance_velocity_controller");
	ros::NodeHandle n;
	ros::Subscriber CommandSub = n.subscribe("/dasom/reference_position", 10, commandcallback);  // Command From velocity_Admittance_Test.cpp
	ros::Publisher Commandpub = n.advertise<geometry_msgs::Twist>("/dasom/goal_dynamixel_position", 100); // Final Angle Command
	ros::Subscriber JointSub = n.subscribe("/dasom/joint_states", 10, jointcallback); // Effort, Position, Velocity
	ros::Publisher forPub = n.advertise<geometry_msgs::Twist>("/dasom/Test_Topic", 100); // 이것저것 쏘기 위함



//---------------------게인튜닝하자----------------//
	P_gain[0] = 200;
	I_gain[0] = 10;
	D_gain[0] = 1.1;

	P_gain[1] = 200;
	I_gain[1] = 10;
	D_gain[1] = 1.1;


	ros::Rate rate(100);

	geometry_msgs::Twist twist;
	geometry_msgs::Twist command;

	double Link1 = 0.1285;
	double Link2 = 0.108;


	time_i = ros::Time::now().toSec();

	while(ros::ok())
	{




	//----------------dt 구하기 용------------//
		time_f = ros::Time::now().toSec();
		time_loop = time_f - time_i;
		time_i = ros::Time::now().toSec();


		Jacobian = solve_Jacobian(measured_angle[0], measured_angle[1], Link1, Link2);
		Jacobian_inverse = Jacobian.transpose();
//		Jacobian_inverse = solve_Jacobian_Inverse(measured_angle[0], measured_angle[1], Link1, Link2);

		measured_position = Forward_Kinematics(measured_angle[0], measured_angle[1], Link1, Link2);




		command.linear.x = command_position[0];
		command.linear.y = command_position[1];
		command.angular.x = measured_position[0];
		command.angular.y = measured_position[1];




	//---------------포지션 에러 구하기 ---------------//


		for(int i = 0; i<2; i++)
		{
		error_p[i] = command_position[i] - measured_position[i];
		error_i[i] = error_i[i] + error_p[i] * time_loop;
		if (error_p[i] - error_p_i[i] != 0) error_d[i] = (error_p[i] - error_p_i[i]) / time_loop;
		error_p_i[i] = error_p[i];
		
		position_gain[i] = P_gain[i] * error_p[i] + I_gain[i] * error_i[i] + D_gain[i] * error_d[i];
		}


		angular_velocity_command = Jacobian_inverse * position_gain;


		twist.linear.x = angular_velocity_command[0];
		twist.angular.z = angular_velocity_command[1];
		
		Commandpub.publish(twist);


		forPub.publish(command);

		ros::spinOnce();
		rate.sleep();


//	std::cout << Jacobian_inverse << "\n ---------------- \n";
//	std::cout << Jacobian_inverse_ << "\n =================== \n";
	
	ROS_INFO("[cmd_x] : %lf, [cmd_y] : %lf ", command_position[0], command_position[1]);
	ROS_INFO("[FK_x] : %lf, [FK_y] : %lf ", measured_position[0], measured_position[1]);
	ROS_INFO("[vel_x] : %lf, [vel_y] : %lf", angular_velocity_command[0], angular_velocity_command[1]);
	ROS_INFO("[ang1] : %lf, [ang2] : %lf ", measured_angle[0], measured_angle[1]);

	

//	ROS_INFO("%lf, %lf \n", angular_velocity_command[0], angular_velocity_command[1]);
//	ROS_INFO("x error : %lf, y error : %lf", command_position[0] - measured_position[0], command_position[1] - measured_position[1]);


	}
	return 0;



}