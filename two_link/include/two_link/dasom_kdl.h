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

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames_io.hpp>
#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/JointState.h>

#define PI 3.14159256359

class DSKDL
{
 public:
  DSKDL();
  ~DSKDL();

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray q_;
  KDL::JntArray tau_;
  KDL::Vector grav_;
  KDL::JntArray G_;
  

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher Commandpub;

  //----------Link Lengths---------//
  static double l1;
  static double l2;
  static double l3;
  static double l4;
  static double l5;
  static double l6;
  static double l7;
  
  static unsigned int num_joints = 6; //
  static unsigned int num_links = 6; //

};

double DSKDL::l1 = 0.01;
double DSKDL::l2 = 0.1;
double DSKDL::l3 = 0.1;
double DSKDL::l4 = 0.001;
double DSKDL::l5 = 0.1;
double DSKDL::l6 = 0.001;
double DSKDL::l7 = 0.1;

#endif //DASOM_KDL_H_
