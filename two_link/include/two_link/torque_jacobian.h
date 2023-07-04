#ifndef TORQUE_JACOBIAN_H_
#define TORQUE_JACOBIAN_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>
#include <dynamixel_workbench_msgs/EECommand.h>
#include "two_link/movingFlag.h"
#include "two_link/admittanceTest.h"

#define PI 3.14159256359

class TorqJ
{
 public:
  TorqJ();
  ~TorqJ();

  double X = 0.23209;
  double Y = 0;

//----------Link Lengths---------//
	double Link1 = 0.10375;
	double Link2 = 0.13634;
  double Link3 = 0.104;
  //----------Link Lengths---------//
	double CoM1 = 0.07821;
	double CoM2 = 0.1117;
	double delta = 0.261;
	double mass1 = 0.10772;
	double mass2 = 0.29951;
  double Kt_1 = 1;
  double Kt_2 = 1;



  //--Offset for gravity matrix--//
  double offset_1 = 2.62;
  double offset_2 = 0.;
  double offset_3 = 0.0;


  Eigen::Vector2d X_ref;
  Eigen::Vector2d X_cmd;
  Eigen::Vector2d X_Command;
  Eigen::Vector2d X_measured;
  Eigen::Vector2d V_measured;
  Eigen::Vector3d angle_measured;
  Eigen::Vector2d theta_dot;


  Eigen::Vector2d X_error_p;
  Eigen::Vector2d X_error_p_i;
  Eigen::Vector2d X_error_i;
  Eigen::Vector2d X_error_d;


  Eigen::Vector2d V_error_p;
  Eigen::Vector2d V_error_p_i;
  Eigen::Vector2d V_error_i;
  Eigen::Vector2d V_error_d;


  Eigen::Vector2d error_gain;

  Eigen::MatrixXd J;
  Eigen::MatrixXd JT;
  Eigen::MatrixXd JTI;


  Eigen::Vector2d tau_des;
  Eigen::Vector3d angle_ref;
  Eigen::Vector3d tau_gravity;
  Eigen::Vector2d tau_loop;
  Eigen::Vector2d velocity_measured;
  Eigen::Vector2d stiction_gain;
  Eigen::Vector2d FK_EE_pos;
  Eigen::Vector2d POSITION_2;


  //--DoB--//
  Eigen::Vector3d angle_d;
  Eigen::Vector2d angle_d_hat;
  Eigen::Vector2d angle_d_lpf;
  Eigen::Vector2d d_hat;
  Eigen::Vector2d angle_d_safe;
  Eigen::Vector3d angle_command;
  double position_p_gain;
  double position_i_gain;
  double position_d_gain;
  double polar_moment_1;
  double polar_moment_2;
  double theta_d;
  double Cut_Off_Freq2;
  double D;
  double r2;

  Eigen::Vector4d Q_M;
  Eigen::Vector4d Q_M_dot;
  Eigen::Vector4d Q_M_2;
  Eigen::Vector4d Q_M_dot_2;  
  Eigen::Matrix4d Q_M_A;
  Eigen::Vector4d Q_M_B;
  Eigen::Vector4d Q_M_C;

  Eigen::Vector2d Q_angle_d;
  Eigen::Vector2d Q_angle_d_dot;
  Eigen::Vector2d Q_angle_d_2;
  Eigen::Vector2d Q_angle_d_dot_2;
  Eigen::Matrix2d Q_angle_d_A;
  Eigen::Vector2d Q_angle_d_B;
  Eigen::Vector2d Q_angle_d_C;
//--end--//

//--External_Force_Estimation--//
  Eigen::Vector3d tau_measured;
  Eigen::Vector3d hysteresis_max;
  Eigen::Vector3d hysteresis_min;
  Eigen::Vector3d tau_ext;
  Eigen::Vector3d tau_ext_not_deadzone;  
  Eigen::Vector2d Force_ext;
  Eigen::Vector2d Force_ext_not_deadzone;
  Eigen::Vector2d Force_max;
  Eigen::Vector2d Force_min;

//--Admittance Control--//
  Eigen::Matrix2d A_x;
  Eigen::Vector2d B_x;
  Eigen::Vector2d C_x;
  Eigen::Vector2d D_x;

  Eigen::Matrix2d A_y;
  Eigen::Vector2d B_y;
  Eigen::Vector2d C_y;
  Eigen::Vector2d D_y;

  Eigen::Vector2d virtual_mass;
  Eigen::Vector2d virtual_damper;
  Eigen::Vector2d virtual_spring;

  Eigen::Vector2d X_from_model_matrix;
  Eigen::Vector2d X_dot_from_model_matrix;
  Eigen::Vector2d Y_from_model_matrix;
  Eigen::Vector2d Y_dot_from_model_matrix;
  Eigen::Vector2d position_from_model;

  //--For safe--//
  Eigen::Vector3d angle_safe;
  Eigen::Vector3d angle_max;
  Eigen::Vector3d angle_min;
  double virtual_mass_x;
  double virtual_damper_x;
  double virtual_spring_x;


  double virtual_mass_y;
  double virtual_damper_y;
  double virtual_spring_y;



//--for butterworth--//
  Eigen::Vector3d bw_2nd_output;
  Eigen::Vector3d bw_2nd_input;
  Eigen::Vector3d bw_4th_output;
  Eigen::Vector3d bw_4th_input;
  Eigen::Vector2d velocity_filtered;

  Eigen::Vector3d bw2_filtered_current_1_input;
  Eigen::Vector3d bw2_filtered_current_1_output;
  Eigen::Vector3d bw2_filtered_current_2_input;
  Eigen::Vector3d bw2_filtered_current_2_output;
  Eigen::Vector3d bw2_filtered_current_3_input;
  Eigen::Vector3d bw2_filtered_current_3_output;



  Eigen::Vector3d filtered_current;


  double wc;
  double wc2;
  double wc4;
  double wc3;
  double wc_4th;
  double wc2_4th;

  double cut_off_freq;
  double cut_off_freq_current;


  double b0_2nd;
  double b1_2nd;
  double b2_2nd;
  double a0_2nd;
  double a1_2nd;
  double a2_2nd;

  float Cut_Off_Freq;
  //V_gain << 1,1;




//--Service Flag--//
  bool movingFlag = false;




  void calc_des();
  void PublishCmdNMeasured();
  void DoB();
  void Calc_Ext_Force();
  void Admittance_control();
  void angle_safe_func();
  void second_order_butterworth();
  bool movingServiceCallback(two_link::movingFlag::Request  &req,
          two_link::movingFlag::Response &res);
  bool AdmittanceCallback(two_link::admittanceTest::Request  &req,
          two_link::admittanceTest::Response &res);

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  std::string robot_name_;
  sensor_msgs::JointState command_position;
  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher joint_command_pub_;
  ros::Publisher joint_measured_pub_;
  ros::Subscriber joint_states_sub_;
  ros::ServiceServer movingService;
  ros::ServiceServer admitService;




  static Eigen::MatrixXd EE_pos(double theta_1, double theta_2, double theta_3)
{
  //--Calc measured EE_pose--//
	  double l1 = 0.10375;
	  double l2 = 0.13634;
    double l3 = 0.104;
    double cos1 = cos(theta_1);
    double cos2 = cos(theta_2);
    double cos3 = cos(theta_3);
    double sin1 = sin(theta_1);
    double sin2 = sin(theta_2);
    double sin3 = sin(theta_3);
    double cos12 = cos(theta_1 + theta_2);
    double sin12 = sin(theta_1 + theta_2);
    double cos123 = cos(theta_1 + theta_2 + theta_3);
    double sin123 = sin(theta_1 + theta_2 + theta_3);


    Eigen::MatrixXd EE_pos(2,1);

    EE_pos <<
    // X
    l2 * cos12 + l1 * cos1 + l3 * cos123,
    // Y
    l2 * sin12 + l1 * sin1 + l3 * sin123;

    return EE_pos;
};

  static Eigen::MatrixXd Jacobian(double theta_1, double theta_2, double theta_3)
{
	  double l1 = 0.10375;
	  double l2 = 0.13634;
    double l3 = 0.104;
    double cos1 = cos(theta_1);
    double cos2 = cos(theta_2);
    double sin1 = sin(theta_1);
    double sin2 = sin(theta_2);
    double cos12 = cos(theta_1 + theta_2);
    double sin12 = sin(theta_1 + theta_2);
    double sin123 = sin(theta_1 + theta_2 + theta_3);
    double cos123 = cos(theta_1 + theta_2 + theta_3);


    Eigen::MatrixXd J(2,3);

    J <<
    // 1X1
    -l2 * sin12 - l1 * sin1 - l3 * sin123,
    // 1X2
    -l2 * sin12 - l3 * sin123,
    // 1X3
    -l3 * sin123,
    // 2X1
    l2 * cos12 + l1 * cos1 + l3 * cos123,
    // 2X2
    l2 * cos12 + l3 * cos123,
    // 2X3
    l3 * cos123;

    return J;
};



  void poseCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void commandCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  //void EEpositionCallback(const dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res);
  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //TorqJ_H_