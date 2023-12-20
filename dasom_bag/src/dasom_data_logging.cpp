#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>

#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/WrenchStamped.h>

#include "nav_msgs/Odometry.h"

ros::Publisher data_log_publisher;

std_msgs::Float64MultiArray data_log;

geometry_msgs::Vector3 position;
geometry_msgs::Vector3 desired_position;
geometry_msgs::Vector3 attitude;
geometry_msgs::Vector3 desired_attitude;
geometry_msgs::Vector3 linear_velocity;
geometry_msgs::Vector3 desired_linear_velocity;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 desired_force;
geometry_msgs::Vector3 desired_torque;
geometry_msgs::Vector3 center_of_mass;
geometry_msgs::Vector3 bias_gradient;
geometry_msgs::Vector3 filtered_bias_gradient;
geometry_msgs::Vector3 attitude_dob_disturbance;
geometry_msgs::Vector3 external_force;
geometry_msgs::Vector3 external_torque;
geometry_msgs::Vector3 reference_position;
geometry_msgs::Vector3 calculated_force;
geometry_msgs::Vector3 non_bias_external_force;
geometry_msgs::Vector3 adaptive_external_force;
geometry_msgs::Vector3 adaptive_external_torque;
//geometry_msgs::Vector3 MoI;
geometry_msgs::Vector3 force_dhat;
geometry_msgs::Vector3 torque_dhat;
geometry_msgs::Vector3 imu_lin_acc;

// dasom
geometry_msgs::Twist dasom_EE_command;
geometry_msgs::Twist dasom_EE_measured;
geometry_msgs::Twist dasom_global_EE_command;
geometry_msgs::Twist dasom_global_meas_gimbal_EE_pose;
sensor_msgs::JointState dasom_meas_effort;
geometry_msgs::WrenchStamped ext_force;
geometry_msgs::Twist admittance_X_ref;


double PWM_cmd[8]={1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.};
double individual_motor_thrust[4]={0.0, 0.0, 0.0, 0.0};
double battery_voltage=22.2;
double delta_t=0.0;
double SBUS[10]={1000.,1000.,1000.,1000.,1000.,1000.,1000.,1000.,1000.,1000.};
double theta1=0.0; 
double theta2=0.0;
double theta3=0.0;
double theta4=0.0;
double desired_theta1=0.0;
double desired_theta2=0.0;
double desired_theta3=0.0;
double desired_theta4=0.0;
double mhe_delta_t=0.0;
double mass=7.4;
double adaptive_mhe_delta_t=0.0;

// dasom
double dasom_meas_effort0 = 0;
double dasom_meas_effort1 = 0;
double dasom_meas_effort2 = 0;
double dasom_meas_effort3 = 0;
double dasom_meas_effort4 = 0;
double dasom_meas_effort5 = 0;

double dasom_des_position0 = 0;
double dasom_des_position1 = 0;
double dasom_des_position2 = 0;
double dasom_des_position3 = 0;
double dasom_des_position4 = 0;
double dasom_des_position5 = 0;

double dasom_meas_position0 = 0;
double dasom_meas_position1 = 0;
double dasom_meas_position2 = 0;
double dasom_meas_position3 = 0;
double dasom_meas_position4 = 0;
double dasom_meas_position5 = 0;

void pos_callback(const geometry_msgs::Vector3& msg);
void desired_pos_callback(const geometry_msgs::Vector3& msg);
void attitude_callback(const geometry_msgs::Vector3& msg);
void desired_attitude_callback(const geometry_msgs::Vector3& msg);
void linear_velocity_callback(const geometry_msgs::Vector3& msg);
void desired_linear_velocity_callback(const geometry_msgs::Vector3& msg);
void pwm_cmd_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
void motor_thrust_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void servo_angle_callback(const sensor_msgs::JointState& msg);
void desired_servo_angle_callback(const sensor_msgs::JointState& msg);
void battery_voltage_callback(const std_msgs::Float32& msg);
void sampling_time_callback(const std_msgs::Float32& msg);
void desired_force_callback(const geometry_msgs::Vector3& msg);
void desired_torque_callback(const geometry_msgs::Vector3& msg);
void center_of_mass_callback(const geometry_msgs::Vector3& msg);
void bias_gradient_callback(const geometry_msgs::Vector3& msg);
void filtered_bias_gradient_callback(const geometry_msgs::Vector3& msg);
void sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
void angular_velocity_callback(const geometry_msgs::Vector3& msg);
void attitude_dob_disturbance_callback(const geometry_msgs::Vector3& msg);
void external_force_callback(const geometry_msgs::Vector3& msg);
void external_torque_callback(const geometry_msgs::Vector3& msg);
void mhe_delta_t_callback(const std_msgs::Float32& msg);
void reference_position_callback(const geometry_msgs::Vector3& msg);
void calculated_force_callback(const geometry_msgs::Vector3& msg);
void non_bias_external_force_callback(const geometry_msgs::Vector3& msg);
//void mass_callback(const std_msgs::Float32& msg);
void adaptive_external_force_callback(const geometry_msgs::Vector3& msg);
void adaptive_external_torque_callback(const geometry_msgs::Vector3& msg);
void adaptive_mhe_delta_t_callback(const std_msgs::Float32& msg);
//void MoI_callback(const geometry_msgs::Vector3& msg);
void force_dhat_callback(const geometry_msgs::Vector3& msg);
void torque_dhat_callback(const geometry_msgs::Vector3& msg);
void imu_lin_acc_callback(const geometry_msgs::Vector3& msg);
void publisherSet();

// dasom
void dasom_EE_cmd_callback(const geometry_msgs::Twist &msg);
void dasom_EE_meas_callback(const geometry_msgs::Twist &msg);
void dasom_global_EE_command_callback(const geometry_msgs::Twist &msg);
void dasom_global_meas_gimbal_EE_pose_callback(const geometry_msgs::Twist &msg);
void dasom_meas_effort_callback(const sensor_msgs::JointState &msg);
void dasom_desired_angle_callback(const sensor_msgs::JointState &msg);
void dasom_external_force_callback(const geometry_msgs::WrenchStamped &msg);
void dasom_admittance_X_ref_callback(const geometry_msgs::Twist &msg);


int main(int argc, char **argv)
{
	ros::init(argc, argv,"dasom_data_logging");
	
	ros::NodeHandle nh;
	ros::Subscriber attitude_log=nh.subscribe("/angle",1,attitude_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_attitude_log=nh.subscribe("/desired_angle",1,desired_attitude_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber position_log=nh.subscribe("/pos",1,pos_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_position_log=nh.subscribe("/pos_d",1,desired_pos_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber servo_angle_log=nh.subscribe("/joint_states",1,servo_angle_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_servo_angle_log=nh.subscribe("/goal_dynamixel_position",1,desired_servo_angle_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber pwm_log=nh.subscribe("/PWMs",1,pwm_cmd_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_torque_log=nh.subscribe("/torque_d",1,desired_torque_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_force_log=nh.subscribe("/force_d",1,desired_force_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber motor_thrust_log=nh.subscribe("/Forces",1,motor_thrust_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber sbus_log=nh.subscribe("/sbus",1,sbus_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber delta_t_log=nh.subscribe("/delta_t",1,sampling_time_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_voltage_log=nh.subscribe("/battery_voltage",1,battery_voltage_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber linear_velocity_log=nh.subscribe("/lin_vel",1,linear_velocity_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber desired_linear_velocity_log=nh.subscribe("/lin_vel_d",1,desired_linear_velocity_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber Center_of_Mass_log=nh.subscribe("/Center_of_Mass",1,center_of_mass_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber bias_gradient_log=nh.subscribe("/bias_gradient",1,bias_gradient_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber filtered_bias_gradient_log=nh.subscribe("/filtered_bias_gradient",1,filtered_bias_gradient_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber angular_velocity_log=nh.subscribe("/angular_velocity",1,angular_velocity_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber attitude_dob_disturbance_log=nh.subscribe("/att_dhat",1,attitude_dob_disturbance_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber external_force_log=nh.subscribe("/external_force",1,external_force_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber external_torque_log=nh.subscribe("/external_torque",1,external_torque_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber mhe_delta_t_log=nh.subscribe("/mhe_delta_t",1,mhe_delta_t_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber reference_position_log=nh.subscribe("/reference_position",1,reference_position_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber calculated_force_log=nh.subscribe("/calculated_force",1,calculated_force_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber non_bias_external_force_log=nh.subscribe("/non_bias_external_force",1,non_bias_external_force_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber force_dhat_sub=nh.subscribe("/force_dhat",1,force_dhat_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber torque_dhat_sub=nh.subscribe("/torque_dhat",1,torque_dhat_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber imu_lin_acc_sub=nh.subscribe("/imu_lin_acl",1,imu_lin_acc_callback, ros::TransportHints().tcpNoDelay());

	// dasom
	ros::Subscriber dasom_EE_cmd_position_sub=nh.subscribe("/dasom/EE_command",1,dasom_EE_cmd_callback, ros::TransportHints().tcpNoDelay()); // ?
	ros::Subscriber dasom_EE_meas_position_sub=nh.subscribe("/dasom/EE_pose",1,dasom_EE_meas_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber dasom_global_EE_cmd_sub=nh.subscribe("/dasom/tf/global_EE_cmd", 1, dasom_global_EE_command_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber dasom_global_meas_gimbal__EE_pose_sub=nh.subscribe("/dasom/tf/global_EE_meas_pose", 1, dasom_global_meas_gimbal_EE_pose_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber dasom_measured_effort_sub=nh.subscribe("/dasom/joint_states", 1, dasom_meas_effort_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber dasom_desired_position_sub=nh.subscribe("/dasom/goal_dynamixel_position", 1, dasom_desired_angle_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber dasom_external_force_sub=nh.subscribe("/dasom/external_force", 1, dasom_external_force_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber dasom_admittance_X_ref_sub=nh.subscribe("/dasom/test_Pub2", 1, dasom_admittance_X_ref_callback, ros::TransportHints().tcpNoDelay());

	data_log_publisher=nh.advertise<std_msgs::Float64MultiArray>("data_log",10);
	ros::Timer timerPulish_log=nh.createTimer(ros::Duration(1.0/200.0), std::bind(publisherSet));
	ros::spin();
	return 0;
}

void publisherSet()
{
	data_log.data.resize(124);

	// data_log.data[0]=attitude.x;
	// data_log.data[1]=attitude.y;
	// data_log.data[2]=attitude.z;
	// data_log.data[3]=desired_attitude.x;
	// data_log.data[4]=desired_attitude.y;
	// data_log.data[5]=desired_attitude.z;
	// data_log.data[6]=position.x;
	// data_log.data[7]=position.y;
	// data_log.data[8]=position.z;
	// data_log.data[9]=desired_position.x;
	// data_log.data[10]=desired_position.y;
	// data_log.data[11]=desired_position.z;
	// data_log.data[12]=theta1;
	// data_log.data[13]=theta2;
	// data_log.data[14]=theta3;
	// data_log.data[15]=theta4;
	// data_log.data[16]=desired_theta1;
	// data_log.data[17]=desired_theta2;
	// data_log.data[18]=desired_theta3;
	// data_log.data[19]=desired_theta4;
	// data_log.data[20]=PWM_cmd[0];
	// data_log.data[21]=PWM_cmd[1];
	// data_log.data[22]=PWM_cmd[2];
	// data_log.data[23]=PWM_cmd[3];
	// data_log.data[24]=PWM_cmd[4];
	// data_log.data[25]=PWM_cmd[5];
	// data_log.data[26]=PWM_cmd[6];
	// data_log.data[27]=PWM_cmd[7];
	// data_log.data[28]=desired_torque.x;
	// data_log.data[29]=desired_torque.y;
	// data_log.data[30]=desired_torque.z;
	// data_log.data[31]=desired_force.x;
	// data_log.data[32]=desired_force.y;
	// data_log.data[33]=desired_force.z;
	// data_log.data[34]=individual_motor_thrust[0];
	// data_log.data[35]=individual_motor_thrust[1];
	// data_log.data[36]=individual_motor_thrust[2];
	// data_log.data[37]=individual_motor_thrust[3];
	// data_log.data[38]=SBUS[0];
	// data_log.data[39]=SBUS[1];
	// data_log.data[40]=SBUS[2];
	// data_log.data[41]=SBUS[3];
	// data_log.data[42]=SBUS[4];
	// data_log.data[43]=SBUS[5];
	// data_log.data[44]=SBUS[6];
	// data_log.data[45]=SBUS[7];
	// data_log.data[46]=SBUS[8];
	// data_log.data[47]=delta_t;
	// data_log.data[48]=battery_voltage;
	// data_log.data[49]=linear_velocity.x;
	// data_log.data[50]=linear_velocity.y;
	// data_log.data[51]=linear_velocity.z;
	// data_log.data[52]=desired_linear_velocity.x;
	// data_log.data[53]=desired_linear_velocity.y;
	// data_log.data[54]=desired_linear_velocity.z;
	// data_log.data[55]=center_of_mass.x;
	// data_log.data[56]=center_of_mass.y;
	// data_log.data[57]=center_of_mass.z;
	// data_log.data[58]=bias_gradient.x;
	// data_log.data[59]=bias_gradient.y;
	// data_log.data[60]=bias_gradient.z;
	// data_log.data[61]=filtered_bias_gradient.x;
	// data_log.data[62]=filtered_bias_gradient.y;
	// data_log.data[63]=filtered_bias_gradient.z;
	// data_log.data[64]=angular_velocity.x;
	// data_log.data[65]=angular_velocity.y;
	// data_log.data[66]=angular_velocity.z;
	// data_log.data[67]=attitude_dob_disturbance.x;
	// data_log.data[68]=attitude_dob_disturbance.y;
	// data_log.data[69]=attitude_dob_disturbance.z;
	// data_log.data[70]=external_force.x;
	// data_log.data[71]=external_force.y;
	// data_log.data[72]=external_force.z;
	// data_log.data[73]=external_torque.x;
	// data_log.data[74]=external_torque.y;
	// data_log.data[75]=external_torque.z;
	// data_log.data[76]=mhe_delta_t;	
	// data_log.data[77]=reference_position.x;
	// data_log.data[78]=reference_position.y;
	// data_log.data[79]=reference_position.z;
	// data_log.data[80]=calculated_force.x;
	// data_log.data[81]=calculated_force.y;
	// data_log.data[82]=calculated_force.z;
	// data_log.data[83]=non_bias_external_force.x;	
	// data_log.data[84]=non_bias_external_force.y;	
	// data_log.data[85]=non_bias_external_force.z;	
	// data_log.data[86]=mass;
	// data_log.data[87]=force_dhat.x;
	// data_log.data[88]=force_dhat.y;
	// data_log.data[89]=force_dhat.z;
	// data_log.data[90]=torque_dhat.x;
	// data_log.data[91]=torque_dhat.y;
	// data_log.data[92]=torque_dhat.z;
	// data_log.data[93]=imu_lin_acc.x;	
	// data_log.data[94]=imu_lin_acc.y;	
	// data_log.data[95]=imu_lin_acc.z;	

	// dasom
	data_log.data[0] = dasom_EE_command.linear.x;
	data_log.data[1] = dasom_EE_command.linear.y;
	data_log.data[2] = dasom_EE_command.linear.z;
	data_log.data[3] = dasom_EE_command.angular.x;
	data_log.data[4] = dasom_EE_command.angular.y;
	data_log.data[5] = dasom_EE_command.angular.z;

	data_log.data[6] = dasom_EE_measured.linear.x;
	data_log.data[7] = dasom_EE_measured.linear.y;
	data_log.data[8] = dasom_EE_measured.linear.z;
	data_log.data[9] = dasom_EE_measured.angular.x;
	data_log.data[10] = dasom_EE_measured.angular.y;
	data_log.data[11] = dasom_EE_measured.angular.z;

	data_log.data[12] = dasom_global_EE_command.linear.x;
	data_log.data[13] = dasom_global_EE_command.linear.y;
	data_log.data[14] = dasom_global_EE_command.linear.z;
	data_log.data[15] = dasom_global_EE_command.angular.x;
	data_log.data[16] = dasom_global_EE_command.angular.y;
	data_log.data[17] = dasom_global_EE_command.angular.z;

	data_log.data[18] = dasom_global_meas_gimbal_EE_pose.linear.x;
	data_log.data[19] = dasom_global_meas_gimbal_EE_pose.linear.y;
	data_log.data[20] = dasom_global_meas_gimbal_EE_pose.linear.z;
	data_log.data[21] = dasom_global_meas_gimbal_EE_pose.angular.x;
	data_log.data[22] = dasom_global_meas_gimbal_EE_pose.angular.y;
	data_log.data[23] = dasom_global_meas_gimbal_EE_pose.angular.z;

	data_log.data[24] = dasom_meas_effort0;
	data_log.data[25] = dasom_meas_effort1;
	data_log.data[26] = dasom_meas_effort2;
	data_log.data[27] = dasom_meas_effort3;
	data_log.data[28] = dasom_meas_effort4;
	data_log.data[29] = dasom_meas_effort5;

	data_log.data[30] = dasom_des_position0;
	data_log.data[31] = dasom_des_position1;
	data_log.data[32] = dasom_des_position2;
	data_log.data[33] = dasom_des_position3;
	data_log.data[34] = dasom_des_position4;
	data_log.data[35] = dasom_des_position5;

	data_log.data[36] = dasom_meas_position0;
	data_log.data[37] = dasom_meas_position1;
	data_log.data[38] = dasom_meas_position2;
	data_log.data[39] = dasom_meas_position3;
	data_log.data[40] = dasom_meas_position4;
	data_log.data[41] = dasom_meas_position5;

	// palletrone
	data_log.data[42] = PWM_cmd[0];
	data_log.data[43] = PWM_cmd[1];
	data_log.data[44] = PWM_cmd[2];
	data_log.data[45] = PWM_cmd[3];
	data_log.data[46] = PWM_cmd[4];
	data_log.data[47] = PWM_cmd[5];
	data_log.data[48] = PWM_cmd[6];
	data_log.data[49] = PWM_cmd[7];

	data_log.data[50] = battery_voltage;

	data_log.data[51] = attitude.x;
	data_log.data[52] = attitude.y;
	data_log.data[53] = attitude.z;
	data_log.data[54] = desired_attitude.x;
	data_log.data[55] = desired_attitude.y;
	data_log.data[56] = desired_attitude.z;
	data_log.data[57] = position.x;
	data_log.data[58] = position.y;
	data_log.data[59] = position.z;
	data_log.data[60] = desired_position.x;
	data_log.data[61] = desired_position.y;
	data_log.data[62] = desired_position.z;

	// dasom
	data_log.data[63] = ext_force.wrench.force.x;
	data_log.data[64] = ext_force.wrench.force.y;
	data_log.data[65] = ext_force.wrench.force.z;
	data_log.data[66] = ext_force.wrench.torque.x;
	data_log.data[67] = ext_force.wrench.torque.y;
	data_log.data[68] = ext_force.wrench.torque.z;

	data_log.data[69] = admittance_X_ref.linear.x;
	data_log.data[70] = admittance_X_ref.linear.y;
	data_log.data[71] = admittance_X_ref.linear.z;
	data_log.data[72] = admittance_X_ref.angular.x;
	data_log.data[73] = admittance_X_ref.angular.y;
	data_log.data[74] = admittance_X_ref.angular.z;

	data_log_publisher.publish(data_log);
}

void pos_callback(const geometry_msgs::Vector3& msg){
	position.x=msg.x;
	position.y=msg.y;
	position.z=msg.z;
}

void desired_pos_callback(const geometry_msgs::Vector3& msg){
	desired_position.x=msg.x;
	desired_position.y=msg.y;
	desired_position.z=msg.z;
}

void attitude_callback(const geometry_msgs::Vector3& msg){
	attitude.x=msg.x;
	attitude.y=msg.y;
	attitude.z=msg.z;
}

void desired_attitude_callback(const geometry_msgs::Vector3& msg){
	desired_attitude.x=msg.x;
	desired_attitude.y=msg.y;
	desired_attitude.z=msg.z;
}

void servo_angle_callback(const sensor_msgs::JointState& msg){
	theta1=msg.position[0];
	theta2=msg.position[1];
	theta3=msg.position[2];
	theta4=msg.position[3];
}

void desired_servo_angle_callback(const sensor_msgs::JointState& msg){
	desired_theta1=msg.position[0];
	desired_theta2=msg.position[1];
	desired_theta3=msg.position[2];
	desired_theta4=msg.position[3];
}

void pwm_cmd_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	for(int i=0;i<8;i++){
		PWM_cmd[i]=msg->data[i];
	}
}

void motor_thrust_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(int i=0;i<4;i++){
		individual_motor_thrust[i]=msg->data[i];
	}	
}

void linear_velocity_callback(const geometry_msgs::Vector3& msg){
	linear_velocity.x=msg.x;
	linear_velocity.y=msg.y;
	linear_velocity.z=msg.z;
}


void desired_linear_velocity_callback(const geometry_msgs::Vector3& msg){
	desired_linear_velocity.x=msg.x;
	desired_linear_velocity.y=msg.y;
	desired_linear_velocity.z=msg.z;
}

void battery_voltage_callback(const std_msgs::Float32& msg){
	battery_voltage=msg.data;	
}

void sampling_time_callback(const std_msgs::Float32& msg){
	delta_t=msg.data;
}

void desired_force_callback(const geometry_msgs::Vector3& msg){
	desired_force.x=msg.x;
	desired_force.y=msg.y;
	desired_force.z=msg.z;
}

void desired_torque_callback(const geometry_msgs::Vector3& msg){
	desired_torque.x=msg.x;
	desired_torque.y=msg.y;
	desired_torque.z=msg.z;
}

void center_of_mass_callback(const geometry_msgs::Vector3& msg){
	center_of_mass.x=msg.x;
	center_of_mass.y=msg.y;
	center_of_mass.z=msg.z;
}

void bias_gradient_callback(const geometry_msgs::Vector3& msg){
	bias_gradient.x=msg.x;
	bias_gradient.y=msg.y;
	bias_gradient.z=msg.z;
}

void filtered_bias_gradient_callback(const geometry_msgs::Vector3& msg){
	filtered_bias_gradient.x=msg.x;
	filtered_bias_gradient.y=msg.y;
	filtered_bias_gradient.z=msg.z;
}

void sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	for(int i=0;i<10;i++){
		SBUS[i]=msg->data[i];	
	}
}

void angular_velocity_callback(const geometry_msgs::Vector3& msg){
	angular_velocity=msg;
}

void attitude_dob_disturbance_callback(const geometry_msgs::Vector3& msg){
	attitude_dob_disturbance=msg;
}

void external_force_callback(const geometry_msgs::Vector3& msg){
	external_force=msg;
}

void external_torque_callback(const geometry_msgs::Vector3& msg){
	external_torque=msg;
}

void mhe_delta_t_callback(const std_msgs::Float32& msg){
	mhe_delta_t=msg.data;
}

void reference_position_callback(const geometry_msgs::Vector3& msg){
	reference_position=msg;
}

void calculated_force_callback(const geometry_msgs::Vector3& msg){
	calculated_force=msg;
}

void non_bias_external_force_callback(const geometry_msgs::Vector3& msg){
	non_bias_external_force=msg;
}

void mass_callback(const std_msgs::Float32& msg){
	mass=msg.data;
}

void adaptive_external_force_callback(const geometry_msgs::Vector3& msg){
	adaptive_external_force=msg;
}

void adaptive_external_torque_callback(const geometry_msgs::Vector3& msg){
	adaptive_external_torque=msg;
}

void adaptive_mhe_delta_t_callback(const std_msgs::Float32& msg){
	adaptive_mhe_delta_t=msg.data;
}

/*void MoI_callback(const geometry_msgs::Vector3& msg){
	MoI=msg;
}*/

void force_dhat_callback(const geometry_msgs::Vector3& msg){
	force_dhat.x=msg.x;
	force_dhat.y=msg.y;
	force_dhat.z=msg.z;
}

void torque_dhat_callback(const geometry_msgs::Vector3& msg){
	torque_dhat=msg;
}

void imu_lin_acc_callback(const geometry_msgs::Vector3& msg){
	imu_lin_acc=msg;
}

// dasom
void dasom_EE_cmd_callback(const geometry_msgs::Twist &msg)
{
	dasom_EE_command = msg;
}

void dasom_EE_meas_callback(const geometry_msgs::Twist &msg)
{
	dasom_EE_measured = msg;
}

void dasom_global_EE_command_callback(const geometry_msgs::Twist &msg)
{
	dasom_global_EE_command = msg;
}

void dasom_global_meas_gimbal_EE_pose_callback(const geometry_msgs::Twist &msg)
{
	dasom_global_meas_gimbal_EE_pose = msg;
}

void dasom_meas_effort_callback(const sensor_msgs::JointState &msg)
{
	dasom_meas_effort0 = msg.effort[0];
	dasom_meas_effort1 = msg.effort[1];
	dasom_meas_effort2 = msg.effort[2];
	dasom_meas_effort3 = msg.effort[3];
	dasom_meas_effort4 = msg.effort[4];
	dasom_meas_effort5 = msg.effort[5];

	dasom_meas_position0 = msg.position[0];
	dasom_meas_position1 = msg.position[1];
	dasom_meas_position2 = msg.position[2];
	dasom_meas_position3 = msg.position[3];
	dasom_meas_position4 = msg.position[4];
	dasom_meas_position5 = msg.position[5];
}

void dasom_desired_angle_callback(const sensor_msgs::JointState &msg)
{
	dasom_des_position0 = msg.position[0];
	dasom_des_position1 = msg.position[1];
	dasom_des_position2 = msg.position[2];
	dasom_des_position3 = msg.position[3];
	dasom_des_position4 = msg.position[4];
	dasom_des_position5 = msg.position[5];
}

void dasom_external_force_callback(const geometry_msgs::WrenchStamped &msg)
{
	ext_force = msg;
}

void dasom_admittance_X_ref_callback(const geometry_msgs::Twist &msg)
{
	admittance_X_ref = msg;
}