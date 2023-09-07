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

#ifndef DASOM_WORKBENCH_H_
#define DASOM_WORKBENCH_H_

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>

#define PI 3.14159256359

namespace dasom
{
  class DasomWorkbench
  {
  public:
    DasomWorkbench();
    ~DasomWorkbench();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    // For MCG Dynamics
    KDL::JntSpaceInertiaMatrix H;
    KDL::JntArray C;
    KDL::JntArray G;

    // For admittance control
    double virtual_mass_x;
    double virtual_damper_x;
    double virtual_spring_x;
    double virtual_mass_y;
    double virtual_damper_y;
    double virtual_spring_y;
    double virtual_mass_z;
    double virtual_damper_z;
    double virtual_spring_z;

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/ 
    void test();  
    void KDLrun(Eigen::VectorXd angle, Eigen::VectorXd velocity);
    void initializeRobotLinks();
    void computeMCGDynamics();
    void initializeAdmittance();
    double admittanceControlX(double time_loop, double ref, double f_ext);
    double admittanceControlY(double time_loop, double ref, double f_ext);
    double admittanceControlZ(double time_loop, double ref, double f_ext);

    Eigen::MatrixXd EE_pose(Eigen::VectorXd measured_angle);
    Eigen::MatrixXd Jacobian(Eigen::VectorXd measured_angle);
    Eigen::Matrix3d CmdOrientation(double roll, double pitch, double yaw);
    Eigen::VectorXd InverseKinematics(Eigen::VectorXd EndEffector_cmd);

  private:
    /*****************************************************************************
    ** ROS NodeHandle
    *****************************************************************************/
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    // For KDL
    KDL::Chain kdl_chain_; // 로봇팔 체인
    KDL::JntArray q_;      // 관절 위치
    KDL::JntArray q_dot_;  // 관절 속도
    KDL::JntArray q_dotdot_; // 관절 가속도

    // For admittance control
    Eigen::Matrix2d A_x;
    Eigen::Vector2d B_x;
    Eigen::Vector2d C_x;
    Eigen::Vector2d D_x;
    Eigen::Vector2d X_from_model_matrix;
    Eigen::Vector2d X_dot_from_model_matrix;

    Eigen::Matrix2d A_y;
    Eigen::Vector2d B_y;
    Eigen::Vector2d C_y;
    Eigen::Vector2d D_y;
    Eigen::Vector2d Y_from_model_matrix;
    Eigen::Vector2d Y_dot_from_model_matrix;

    Eigen::Matrix2d A_z;
    Eigen::Vector2d B_z;
    Eigen::Vector2d C_z;
    Eigen::Vector2d D_z;
    Eigen::Vector2d Z_from_model_matrix;
    Eigen::Vector2d Z_dot_from_model_matrix;

    // For kinematics
    // link lengths
    double l1;
    double l2;
    double l3;
    double l4;
    double l5;
    double l6;
    double l7;

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
    Eigen::Matrix4d L1(double theta_1);
    Eigen::Matrix4d L2(double theta_2);
    Eigen::Matrix4d L3(double theta_3);
    Eigen::Matrix4d L4(double theta_4);
    Eigen::Matrix4d L5();
    Eigen::Matrix4d L6(double theta_5);
    Eigen::Matrix4d L7(double theta_6);
    Eigen::Matrix3d R03(double theta_1, double theta_2, double theta_3);
  };
} // namespace DASOM

#endif /*DASOM_WORKBENCH_H_*/
