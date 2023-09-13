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

#include "../../include/dasom_toolbox/dasom_palletrone.h"

using namespace dasom;

DasomPalletrone::DasomPalletrone()
: nh_(""), priv_nh_("")
{

}

DasomPalletrone::~DasomPalletrone()
{
  ROS_INFO("Bye DasomPalletrone!");
  ros::shutdown();
}

void DasomPalletrone::test()
{
  ROS_INFO("HI! Here is DasomPalletrone!");
}

Eigen::Matrix3d DasomPalletrone::RotX(double theta)
{
  Eigen::Matrix3d RotX;

  RotX <<     1,             0,             0,
              0,         cos(theta),   -sin(theta),
              0,         sin(theta),    cos(theta);

  return RotX;
}

Eigen::Matrix3d DasomPalletrone::RotZ(double theta)
{
  Eigen::Matrix3d RotZ;

  RotZ << cos(theta),   -sin(theta),        0,
          sin(theta),    cos(theta),        0,
              0,             0,             1;

  return RotZ;
}

Eigen::Matrix3d DasomPalletrone::RotD2M()
{
  Eigen::Matrix3d RotD2M;

  RotD2M << RotZ(PI/4) * RotX(PI);

  return RotD2M;
}

Eigen::Vector3d DasomPalletrone::M2Dposition(Eigen::Vector3d Mposition)
{
  Eigen::Vector3d M2Dposition;

  M2Dposition << RotD2M() * Mposition;

  return M2Dposition;
}

Eigen::Vector3d DasomPalletrone::D2Mposition(Eigen::Vector3d Dposition)
{
  Eigen::Vector3d D2Mposition;

  D2Mposition << RotD2M().inverse() * Dposition;

  return D2Mposition;
}