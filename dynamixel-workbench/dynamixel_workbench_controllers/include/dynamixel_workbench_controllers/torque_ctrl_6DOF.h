/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_TORQUE_CTRL_6DOF_H
#define DYNAMIXEL_WORKBENCH_TORQUE_CTRL_6DOF_H

#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "message_header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <iostream>

class TorqueControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;


  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;
  ros::Publisher forwardkinematics_pub_;
  ros::Publisher test_pub_;

  // ROS Topic Subscriber
  ros::Subscriber joint_command_sub_;
  ros::Subscriber paletrone_sub_;
  // ROS Service Server

  // ROS Service Client

  // Mode Change Parameters
  std::vector<float> max_current = {1193, 1193, 1193, 1193, 1193, 1193};
  std::vector<float> limit_current = {1193, 1193, 1193, 1193, 1193, 1193};

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

  float p_gain_;
  float d_gain_;

  bool stopFlag = false;

  Eigen::Vector2d EE_position;

  int32_t goal_position_[16];
  double goal_position[16];
  double present_position_[16];
  double present_position_i[16] = {0, };
  double num_deriv[16] = {0, };
  int32_t goal_dxl_position[16];

  double base_joint_X = 0;
  double base_joint_Y = 0;
  double base_joint_Z = 0;
  double base_joint_r = 0;
  double base_joint_p = 0;
  double base_joint_y = 0;

 public:
  TorqueControl();
  ~TorqueControl();

  void jointStatePublish();
  void Test();

  double time_loop = 0;
  double time_i = 0;
  double time_f = 0;
  void initpose(); //이름은 알아서 바꾸셔요!

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();

  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);

  void paletroneCallback(const geometry_msgs::PoseStamped &msg);



};

#endif //DYNAMIXEL_WORKBENCH_TORQUE_CTRL_6DOF_H
