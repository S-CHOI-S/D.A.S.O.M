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

#include "../../include/dasom_toolbox/dasom_lpf.h"

DasomLPF::DasomLPF(double cut_off_freq)
: nh_("")
{
  ROS_INFO("Dasom LPF Node Start!");

  cof = cut_off_freq;
  
  bw2_filtered_input.resize(3);
  bw2_filtered_output.resize(3);

  bw2_filtered_input << 0, 0, 0;
  bw2_filtered_output << 0, 0, 0;
}

DasomLPF::~DasomLPF()
{
  
}

void DasomLPF::test()
{
  ROS_INFO("Here is Dasom LPF Toolbox!");
}

double DasomLPF::updateLPF(double time_loop, double input_data)
{
  wc = tan(PI * cof * time_loop);
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