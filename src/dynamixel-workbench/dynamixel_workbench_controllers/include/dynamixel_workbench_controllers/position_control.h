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

#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H

#include <ros/ros.h>

#include "message_header.h"

#include <sensor_msgs/JointState.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
// ADD
#include <dynamixel_workbench_msgs/DasomDynamixel.h>

class PositionControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;
  ros::Publisher dasom_joint_states_pub_;
  ros::Publisher TestPub; // ADD

  // ROS Topic Subscriber
  ros::Subscriber joint_command_sub_;

  // ROS Service Server
  ros::ServiceServer joint_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  std::string robot_name_; // ADD
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

  // ADD
  bool flag1;
  bool flag2;
  bool flag3;  

 public:
  PositionControl();
  ~PositionControl();
  void controlLoop(void);

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();
  void jointStatePublish();

  void initServer();
  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  void goalJointPositionCallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg);
};

#endif //DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
