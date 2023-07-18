#ifndef DASOM_KDL_H_
#define DASOM_KDL_H_
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

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>

class DasomKDL
{
public:
  DasomKDL();
  ~DasomKDL();

  void run();
  void initializeRobotLinks();
  void computeMCGDynamics();

private:
  ros::NodeHandle nh_;
  // ros::Publisher command_pub_;
  ros::Rate loop_rate_;
  // sensor_msgs::JointState msg_;

  KDL::Chain kdl_chain_; // 로봇팔 체인
  KDL::JntArray q_;      // 관절 위치
  KDL::JntArray q_dot_;  // 관절 속도
  KDL::JntArray q_dotdot_; // 관절 가속도
};

#endif //DASOM_KDL_H_