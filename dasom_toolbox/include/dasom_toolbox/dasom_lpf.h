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

#ifndef DASOM_LPF_H_
#define DASOM_LPF_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define PI 3.141592

class DasomLPF
{
 public:
  DasomLPF(double cut_off_freq);
  ~DasomLPF();

  void test();
  double updateLPF(double time_loop, double input_data);

 private:
  ros::NodeHandle nh_;

  double cof;
  double wc;
  double wc2;
  double a0_2nd;
  double a1_2nd;
  double a2_2nd;
  double b0_2nd;
  double b1_2nd;
  double b2_2nd;
  
  Eigen::VectorXd bw2_filtered_input;
  Eigen::VectorXd bw2_filtered_output;

};

#endif /*DASOM_LPF_H_*/
