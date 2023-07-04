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

//#ifndef DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
//#define DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H

//#include <ros/ros.h>

//#include "message_header.h"
//#include "sensor_msgs/JointState.h"

//#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
//#include <dynamixel_workbench_msgs/DynamixelStateList.h>
//#include <dynamixel_workbench_msgs/JointCommand.h>

//#define PAN 0
//#define TILT 1

//#define PI 3.14159256359

//class TorqueControl
//{
// public:
//    int32_t goal_torque_[2]; // ADD
//
// private:
//  // ROS NodeHandle
//  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
//  ros::Publisher dynamixel_state_list_pub_;
//  ros::Publisher joint_states_pub_;
//  ros::Publisher goal_torque_pub_;

  // ROS Topic Subscriber
//  ros::Subscriber joint_command_sub_;

  // ROS Service Server
//  ros::ServiceServer joint_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
//  DynamixelWorkbench *dxl_wb_;
//  uint8_t dxl_id_[2];
//  uint8_t dxl_cnt_;

//  float p_gain_;
//  float d_gain_;
//  int32_t goal_position_[2];
//  float goal_torque[2];
//  int32_t input_torque[2];


// public:
//  TorqueControl();
//  ~TorqueControl();
//  void controlLoop(void);

// private:
//  void initMsg();

//  void initPublisher();
//  void initSubscriber();
//  void dynamixelStatePublish();
//  void jointStatePublish();

//  void initServer();
//  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
//                               dynamixel_workbench_msgs::JointCommand::Response &res);
//  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
//  void gravityCompensation();
//};

//#endif //DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H

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

#ifndef DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
#define DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H

#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "message_header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

#define PAN 0
#define TILT 1

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

  // ROS Service Server
  ros::ServiceServer STOP_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[2];
  uint8_t dxl_cnt_;

  float p_gain_;
  float d_gain_;

  bool stopFlag = false;

  Eigen::Vector2d EE_position;

  int32_t goal_position_[2];
  double goal_position[2];
  double present_position_[2];
  double present_position_i[2] = {0, 0};
  double num_deriv[2] = {0, 0};
  int32_t goal_dxl_position[2];

 public:
  TorqueControl();
  ~TorqueControl();

  void controlLoop(void);
  void jointStatePublish();
  void ForwardKinematics();
  void safe_func();
  void Test();

  double time_loop = 0;
  double time_i = 0;
  double time_f = 0;

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();

  void initServer();
  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
  bool StopCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  
  static Eigen::MatrixXd EE_pos(double theta_1, double theta_2)
{
    double l1 = 0.10375;
    double l2 = 0.10792;
    double cos1 = cos(theta_1);
    double cos2 = cos(theta_2);
    double sin1 = sin(theta_1);
    double sin2 = sin(theta_2);
    double cos12 = cos(theta_1 + theta_2);
    double sin12 = sin(theta_1 + theta_2);

    Eigen::MatrixXd EE_pos(2,1);

    EE_pos <<
    // X
    l1 * cos1 + l2 * cos12,
    // Y
    l1 * sin1 + l2 * sin12;

    return EE_pos;
};

};

#endif //DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
