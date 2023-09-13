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

#ifndef DASOM_PALLETRONE_H_
#define DASOM_PALLETRONE_H_

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>

#define PI 3.14159256359

namespace dasom
{
  class DasomPalletrone
  {
  public:
    DasomPalletrone();
    ~DasomPalletrone();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/ 
    void test();
    Eigen::Vector3d M2Dposition(Eigen::Vector3d Mposition);
    Eigen::Vector3d D2Mposition(Eigen::Vector3d Dposition);

  private:
    /*****************************************************************************
    ** ROS NodeHandle
    *****************************************************************************/
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    
    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
    Eigen::Matrix3d RotX(double theta);
    Eigen::Matrix3d RotZ(double theta);
    Eigen::Matrix3d RotD2M();
  };
} // namespace DASOM

#endif /*DASOM_PALLETRONE_H_*/
