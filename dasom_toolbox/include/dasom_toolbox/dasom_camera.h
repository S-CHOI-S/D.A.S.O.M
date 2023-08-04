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

#ifndef DASOM_CAMERA_H_
#define DASOM_CAMERA_H_

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>

class DasomWorkbench
{
 public:
  DasomWorkbench();
  ~DasomWorkbench();

  void test();  
  void run();
  void initializeRobotLinks();
  void computeMCGDynamics();

 private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  KDL::Chain kdl_chain_; // 로봇팔 체인
  KDL::JntArray q_;      // 관절 위치
  KDL::JntArray q_dot_;  // 관절 속도
  KDL::JntArray q_dotdot_; // 관절 가속도

  float cnt__ = 0;
  float sec = 0;

};

#endif /*DASOM_CAMERA_H_*/
