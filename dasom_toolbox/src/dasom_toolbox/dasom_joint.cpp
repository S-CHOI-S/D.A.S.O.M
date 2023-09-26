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

#include "../../include/dasom_toolbox/dasom_joint.h"

DasomJoint::DasomJoint(double cut_off_freq_current, double cut_off_freq_qfilter)
: nh_(""), priv_nh_("~")
{
  // For lpf
  cof_current = cut_off_freq_current;
  cof_qfilter = cut_off_freq_qfilter;
  cof_qfilter2 = cof_qfilter * cof_qfilter;
  
  bw2_filtered_input.resize(3);
  bw2_filtered_output.resize(3);
  bw2_filtered_input << 0, 0, 0;
  bw2_filtered_output << 0, 0, 0;

  // For DOB
  P_gain = priv_nh_.param<double>("dob_p_gain",800);
  I_gain = priv_nh_.param<double>("dob_i_gain",0);
  D_gain = priv_nh_.param<double>("dob_d_gain",0);
  polar_moment = priv_nh_.param<double>("dob_polar_moment",0);

  initDOB();
}

DasomJoint::~DasomJoint()
{
  ROS_INFO("Bye DasomJoint!");
  ros::shutdown();
}

void DasomJoint::test()
{
  ROS_INFO("Here is Dasom Joint Toolbox!");
}

double DasomJoint::updateLPF(double time_loop, double input_data)
{
  wc = tan(PI * cof_current * time_loop);
  wc2 = wc*wc;
  b0_2nd = (wc2) / (1 + sqrt(2)*wc + wc2);
  b1_2nd = 2 * b0_2nd;
  b2_2nd = b0_2nd;
  a0_2nd = 1;
  a1_2nd = 2 * (wc2 - 1) / (1 + sqrt(2) * wc + wc2);
  a2_2nd = (1 - sqrt(2) * wc + wc2) / (1 + sqrt(2) * wc + wc2);

  // For 1 joint
  bw2_filtered_input[2] = input_data;

  bw2_filtered_output[2] = b0_2nd * bw2_filtered_input[2] + b1_2nd * bw2_filtered_input[1] + b2_2nd * bw2_filtered_input[0] 
                         - a1_2nd * bw2_filtered_output[1] - a2_2nd * bw2_filtered_output[0];

  bw2_filtered_output[0] = bw2_filtered_output[1];
  bw2_filtered_output[1] = bw2_filtered_output[2];

  bw2_filtered_input[0] = bw2_filtered_input[1];
  bw2_filtered_input[1] = bw2_filtered_input[2];

  return bw2_filtered_output[2];
}

void DasomJoint::initDOB()
{
  // For Model of DOB
  Q_M << 0, 0, 0, 0;
  Q_M_dot << 0, 0, 0, 0;

  Q_M_A << 0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1,
           - I_gain * cof_qfilter2 / D_gain,
           -(P_gain * cof_qfilter2 + sqrt(2) * I_gain * cof_qfilter) / D_gain,
           -(D_gain * cof_qfilter2 + sqrt(2) * P_gain * cof_qfilter + I_gain) / D_gain,
           -(P_gain + sqrt(2) * D_gain * cof_qfilter) / D_gain;

  Q_M_B << 0, 0, 0, 1;

  Q_M_C << cof_qfilter2 * I_gain / D_gain,
           cof_qfilter2 * P_gain / D_gain,
           cof_qfilter2,
           cof_qfilter2 * polar_moment / D_gain;

  // For Q-filter
  Q_angle_d << 0, 0;
  Q_angle_d_dot << 0, 0;
  Q_angle_d_A <<       0,                 1,
                -cof_qfilter2, -sqrt(2) * cof_qfilter;
  Q_angle_d_B << 0, cof_qfilter2;

  Q_angle_d_C << 1, 0;

  Q_angle_d_C = Q_angle_d_C.transpose();
}

double DasomJoint::updateDOB(double time_loop, double angle_measured, double angle_ref)
{
  // For 1 joint
  
  // State space (Gn)
  Q_M_dot = Q_M_A * Q_M + Q_M_B * angle_measured;
  Q_M += Q_M_dot * time_loop;
  angle_d_hat = Q_M_C.dot(Q_M); 

  // State space (theta_d)
  Q_angle_d_dot = Q_angle_d_A * Q_angle_d + Q_angle_d_B * angle_ref;
  Q_angle_d += Q_angle_d_dot * time_loop;
  angle_d_lpf = Q_angle_d_C.dot(Q_angle_d);

  d_hat = angle_d_hat - angle_d_lpf; // d_hat: estimated disturbance

  // std::cout<<"Q_angle_d_dot"<<std::endl<<Q_angle_d_dot<<std::endl;

  // std::cout<<"Q_angle_d"<<std::endl<<Q_angle_d<<std::endl;

  // std::cout<<"angle_d_lpf"<<std::endl<<angle_d_lpf<<std::endl;

  // std::cout<<"angle_ref"<<std::endl<<angle_ref<<std::endl;

  return d_hat;
}