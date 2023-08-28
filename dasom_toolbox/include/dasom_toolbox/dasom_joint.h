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

#ifndef DASOM_Joint_H_
#define DASOM_Joint_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define PI 3.141592

class DasomJoint
{
 public:
  DasomJoint(double cut_off_freq_current, double cut_off_freq_qfilter);
  ~DasomJoint();

  void test();
  double updateLPF(double time_loop, double input_data);
  double updateDOB(double time_loop, double angle_ref, double angle_measured);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  double cof_current;
  double cof_qfilter;
  double cof_qfilter2;
  double wc;
  double wc2;
  double a0_2nd;
  double a1_2nd;
  double a2_2nd;
  double b0_2nd;
  double b1_2nd;
  double b2_2nd;

  double P_gain;
  double I_gain;
  double D_gain;
  double polar_moment;

  double angle_d;
  double angle_d_hat;
  double angle_d_lpf;
  double d_hat;
  
  Eigen::VectorXd bw2_filtered_input;
  Eigen::VectorXd bw2_filtered_output;

  Eigen::Vector4d Q_M;
  Eigen::Vector4d Q_M_dot;
  Eigen::Matrix4d Q_M_A;
  Eigen::Vector4d Q_M_B;
  Eigen::Vector4d Q_M_C;

  Eigen::Vector2d Q_angle_d;
  Eigen::Vector2d Q_angle_d_dot;
  Eigen::Matrix2d Q_angle_d_A;
  Eigen::Vector2d Q_angle_d_B;
  Eigen::Vector2d Q_angle_d_C;

  void initDOB();

};

#endif /*DASOM_JOINT_H_*/
