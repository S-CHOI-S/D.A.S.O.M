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

#ifndef DASOM_MANIPULATOR_CONTROL_H_
#define DASOM_MANIPULATOR_CONTROL_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dasom_controllers/admittanceSRV.h>
#include <dasom_controllers/bandpassSRV.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <dasom_toolbox/dasom_workbench.h>
#include <dasom_toolbox/dasom_joint.h>

using namespace dasom;

class DasomControl : public dasom::DasomWorkbench
{
 public:
  DasomControl();
  ~DasomControl();

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For time loop (count)
  double time_i = 0;
  double time_f = 0;
  double time_loop = 0;

  // For init pose
  bool initPoseFlag;

  // For angle control
  Eigen::VectorXd angle_ref;
  Eigen::VectorXd angle_measured;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void CalcExternalForce();
  void ForceGenerator();
  void DOB();
  void AngleSafeFunction();
  void CommandGenerator();
  void CommandVelocityLimit();
  void SolveInverseKinematics();
  void initPoseFunction();
  void AdmittanceControl();
  void PublishData();
  void test();
  
 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  std::string robot_name_;

  /*****************************************************************************
  ** D.A.S.O.M toolbox
  *****************************************************************************/
  // DasomJoint
  DasomJoint *ds_jnt1_;
  DasomJoint *ds_jnt2_;
  DasomJoint *ds_jnt3_;
  DasomJoint *ds_jnt4_;
  DasomJoint *ds_jnt5_;
  DasomJoint *ds_jnt6_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
  ** ROS Publishers
  *****************************************************************************/
  ros::Publisher joint_command_pub_;
  ros::Publisher joint_measured_pub_;
  ros::Publisher dasom_EE_pos_pub_;
  ros::Publisher gimbal_pub_;
  ros::Publisher force_pub_;
  ros::Publisher test_Pub;
  ros::Publisher test_Pub2;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber joystick_sub_;
  ros::Subscriber button_sub_;
  ros::Subscriber gimbal_cmd_sub_;
  ros::Subscriber global_EE_pose_sub_;

  /*****************************************************************************
  ** ROS Services Clients
  *****************************************************************************/
  ros::ServiceServer admittance_srv_;
  ros::ServiceServer bandpass_srv_;

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For init pose
  Eigen::VectorXd initPose;
  Eigen::VectorXd haptic_offset;
  Eigen::VectorXd initPose_for_initPoseFunction;
  Eigen::VectorXd angle_init;

  // For angle control
  Eigen::VectorXd angle_d;
  
  // For angle safe
  Eigen::VectorXd angle_safe;
  Eigen::VectorXd angle_max;
  Eigen::VectorXd angle_min;

  // For pose control
  Eigen::VectorXd FK_pose;
  Eigen::VectorXd EE_command;
  Eigen::VectorXd EE_command_vel_limit;
  
  // For haptic control
  Eigen::VectorXd haptic_initPose;
  Eigen::VectorXd haptic_command;

  // For haptic button
  int grey_button;
  bool white_button;
  double gripper_cmd;

  // For gimbaling
  Eigen::VectorXd gimbal_tf;
  Eigen::VectorXd gimbal_EE_cmd;
  Eigen::VectorXd global_EE_tf;

  // For gimbaling + command
  int cnt_gimbalcommand = 0;
  bool gimbalcommand_safe = false;
  
  // For force estimation
  Eigen::MatrixXd J;
  Eigen::MatrixXd JT;
  Eigen::MatrixXd JTI;
  Eigen::VectorXd velocity_measured;
  Eigen::VectorXd tau_measured;
  Eigen::VectorXd F_ext;
  Eigen::Vector3d F_max;
  Eigen::Vector3d F_min;
  Eigen::VectorXd hysteresis_max;
  Eigen::VectorXd hysteresis_min;

  // For admittance control
  Eigen::VectorXd X_ref;
  Eigen::VectorXd X_cmd;

  // For DOB
  double dob_cnt = 0;
  Eigen::VectorXd d_hat;

  // For bpf
  double wl;
  double wh;
  double w;
  double Q;
  Eigen::Matrix2d bp_A;
  Eigen::Vector2d bp_B;
  Eigen::Vector2d bp_C;
  double bp_D;
  Eigen::Vector2d bp_X1;
  Eigen::Vector2d bp_X2;
  Eigen::Vector2d bp_X3;
  Eigen::Vector2d bp_X_dot1;
  Eigen::Vector2d bp_X_dot2;
  Eigen::Vector2d bp_X_dot3;
  Eigen::Vector3d bf_F_ext;
  
  // For tanh
  Eigen::Vector3d bf_F_ext_tanh;

  // For generating force
  int cnt_force = 0;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void poseCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void commandCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void joystickCallback(const geometry_msgs::Twist &msg);
  void buttonCallback(const omni_msgs::OmniButtonEvent &msg);
  void globalEEPoseCallback(const geometry_msgs::PoseStamped &msg);
  void gimbalEECmdCallback(const geometry_msgs::PoseStamped &msg);
  bool admittanceCallback(dasom_controllers::admittanceSRV::Request  &req,
                          dasom_controllers::admittanceSRV::Response &res);
  bool bandpassCallback(dasom_controllers::bandpassSRV::Request  &req,
                        dasom_controllers::bandpassSRV::Response &res);
  double tanh_function(double input_data, double cut_off_force);
  void tauLPFforExternalForce();
  void startGimbalHapticCommand();
  void deleteToolbox();
};

#endif //DASOM_MANIPULATOR_CONTROL_H_
