#ifndef TORQUE_JACOBIAN_ICROS_H_
#define TORQUE_JACOBIAN_ICROS_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>
//#include <two_link/Torqbian.h>
#include <dynamixel_workbench_msgs/EECommand.h>

#define PI 3.14159256359

class TorqJ
{
 public:
  TorqJ();
  ~TorqJ();

  double X = 0.21167;
  double Y = 0;

  double X_P_gain = 1;
  double X_I_gain = 1;
  double X_D_gain = 1;

  double Y_P_gain = 1;
  double Y_I_gain = 1;
  double Y_D_gain = 1;

  double VX_P_gain = 1;
  double VX_I_gain = 1;
  double VX_D_gain = 1;

  double VY_P_gain = 1;
  double VY_I_gain = 1;
  double VY_D_gain = 1;



//----------Link Lengths---------//
	double Link1 = 0.10375;
	double Link2 = 0.10792;

  //----------Link Lengths---------//
	double CoM1 = 0.08092;
	double CoM2 = 0.1117;
	double delta = 0.261;
	double mass1 = 0.107;
	double mass2 = 0.288;

  double offset_1 = 0;
  double offset_2 = 0; 

  Eigen::Vector2d X_ref;
  Eigen::Vector2d X_cmd;
  Eigen::Vector2d X_measured;
  Eigen::Vector2d V_measured;
  Eigen::Vector2d angle_measured;
  Eigen::Vector2d theta_dot;

  Eigen::Vector2d Position_P_gain;
  Eigen::Vector2d Position_I_gain;
  Eigen::Vector2d Position_D_gain;

  Eigen::Vector2d Velocity_P_gain;
  Eigen::Vector2d Velocity_I_gain;
  Eigen::Vector2d Velocity_D_gain;

  Eigen::Vector2d X_error_p;
  Eigen::Vector2d X_error_p_i;
  Eigen::Vector2d X_error_i;
  Eigen::Vector2d X_error_d;

  Eigen::Vector2d V_error_p;
  Eigen::Vector2d V_error_p_i;
  Eigen::Vector2d V_error_i;
  Eigen::Vector2d V_error_d;

  Eigen::Vector2d X_PID;
  Eigen::Vector2d V_PID;
  Eigen::Vector2d error_gain;

  Eigen::Matrix2d J;
  Eigen::Matrix2d JT;

  //Eigen::MatrixXd q_dot;
  Eigen::Vector2d tau_des;
  Eigen::Vector2d angle_ref;
  Eigen::Vector2d tau_gravity; //중력에 의해 조인트에 가해지는 토크
  Eigen::Vector2d tau_loop;
  Eigen::Vector2d velocity_measured;
  Eigen::Vector2d stiction_gain;
  Eigen::Vector2d FK_EE_pos;

  //--DoB--//
  Eigen::Vector2d angle_d;
  Eigen::Vector2d angle_d_hat;
  Eigen::Vector2d angle_d_lpf;
  Eigen::Vector2d d_hat;

  double position_p_gain;
  double position_i_gain;
  double position_d_gain;
  double polar_moment_1;
  double polar_moment_2;
  double Cut_Off_Freq2;

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
  Eigen::Vector2d tau_measured;
  Eigen::Vector2d hysteresis_max;
  Eigen::Vector2d hysteresis_min;
  Eigen::Vector2d tau_ext;
  Eigen::Vector2d Force_ext;
  Eigen::Matrix2d JTI;

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




  double wc;
  double wc2;
  double wc4;
  double wc3;
  double wc_4th;
  double wc2_4th;

  double cut_off_freq;
  double cut_off_freq_4th;
  double b0_4th;
  double b1_4th;
  double b2_4th;
  double b3_4th;
  double b4_4th;
  double a0_4th;
  double a1_4th;
  double a2_4th;
  double a3_4th;
  double a4_4th;

  double b0_2nd;
  double b1_2nd;
  double b2_2nd;
  double a0_2nd;
  double a1_2nd;
  double a2_2nd;
  double damping_const;

  float Cut_Off_Freq;
  float Error_Gain;
  //V_gain << 1,1;

//--for friction_compen_pulse--//
  double k_fc;
  double Delta;
  double Pd;
  double fc;
  double w0;
  Eigen::Vector2d u_fc;

//--stiction_compensator--//
  double stiction_alpha;
  double stiction_k;


  void calc_des();
  void calc_taudes();
  void PublishCmdNMeasured();

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
  geometry_msgs::Twist Measured_EE_Position;
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
  ros::Subscriber EE_command_sub_;
  ros::Subscriber forwardkinematics_sub_;
  ros::Subscriber joint_states_sub_;



    static Eigen::MatrixXd EE_pos(double theta_1, double theta_2)
{
    double l1 = 0.10375;
    double l2 = 0.10792;
    double cos1 = cos(theta_1);
    double cos2 = cos(theta_2);
    double sin1 = sin(theta_1);
    double sin2 = sin(theta_2);
    double cos12 = cos(theta_1 + theta_2);
    double sin12 = sin(theta_1 + theta_2);

    Eigen::MatrixXd EE_pos(2,1);

    EE_pos <<
    // X
    l1 * cos1 + l2 * cos12,
    // Y
    l1 * sin1 + l2 * sin12;

    return EE_pos;
};

  static Eigen::MatrixXd Jacobian(double theta_1, double theta_2)
{
    double l1 = 0.12409;
    double l2 = 0.108;
    double cos1 = cos(theta_1);
    double cos2 = cos(theta_2);
    double sin1 = sin(theta_1);
    double sin2 = sin(theta_2);
    double cos12 = cos(theta_1 + theta_2);
    double sin12 = sin(theta_1 + theta_2);

    Eigen::MatrixXd J(2,2);

    J <<
    // 1X1
    -l1 * sin1 - l2 * sin12,
    // 1X2
    -l2 * sin12,
    // 2X1
    l1 * cos1 + l2 * cos12,
    // 2X2
    l2 * cos12;

    return J;
};

  void poseCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void commandCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void second_order_butterworth();
  void fourth_order_butterworth();
  void friction_compen_pulse();
  void second_order_butterworth_();
  void stiction_gravity_compensator();
  void DoB();
  void Calc_Ext_Force();
  void Admittance_control();
  //void EEpositionCallback(const dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res);
  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //TorqJ__ICROS_H_
