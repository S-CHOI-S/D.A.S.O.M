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
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

  float p_gain_;
  float d_gain_;

  bool stopFlag = false;

  Eigen::Vector3d EE_position;

  int32_t goal_position_[16];
  double goal_position[16];
  double present_position_[16];
  double present_position_i[16] = {0, };
  double num_deriv[16] = {0, };
  int32_t goal_dxl_position[16];

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
  
  static Eigen::MatrixXd EE_pos(double theta_1,double theta_2,double theta_3,
			        double theta_4,double theta_5,double theta_6)
  {
      double l1 = 0.04233;
      double l2 = 0.12409;
      double l3 = 0.04794;
      double l4 = 0.06133;
      double l5 = 0.04583;
      double l6 = 0.06733;
      double l7 = 0.1;
      double cos1 = cos(theta_1), sin1 = sin(theta_1);
      double cos2 = cos(theta_2), sin2 = sin(theta_2);
      double cos3 = cos(theta_3), sin3 = sin(theta_3);
      double cos4 = cos(theta_4), sin4 = sin(theta_4);
      double cos5 = cos(theta_5), sin5 = sin(theta_5);
      double cos6 = cos(theta_6), sin6 = sin(theta_6);
      double sin23 = sin(theta_2 + theta_3);
      double cos23 = cos(theta_2 + theta_3);
      double cos45 = cos(theta_4 + theta_5);
      double sin46 = sin(theta_4 + theta_6);
      double cos4m5 = cos(theta_4 - theta_5);
      double sin4m6 = sin(theta_4 - theta_6);
  
      Eigen::MatrixXd EE_pos(3,1);

      EE_pos << 
  // X
  l5*(sin1*sin2*sin3 - cos2*cos3*sin1) - l6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + l4*cos4*(cos2*sin1*sin3 + cos3*sin1*sin2) - l7*cos6*(sin5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) - l2*cos2*sin1 + l4*cos1*sin4 + l7*sin6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - l3*cos2*cos3*sin1 + l3*sin1*sin2*sin3,
  // Y
  l2*cos1*cos2 - l6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l5*(cos1*sin2*sin3 - cos1*cos2*cos3) + l4*sin1*sin4 + l7*sin6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l7*cos6*(sin5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + cos5*(cos1*sin2*sin3 - cos1*cos2*cos3)) - l4*cos4*(cos1*cos2*sin3 + cos1*cos3*sin2) + l3*cos1*cos2*cos3 - l3*cos1*sin2*sin3,
  // Z
  l1 + l3*sin23 + l5*sin23 + l2*sin2 + (l7*cos23*sin46)/2 + l4*cos23*cos4 - l6*cos23*cos4 - (l7*sin4m6*cos23)/2 + l7*cos6*((cos4m5*cos23)/2 - (cos23*cos45)/2 + sin23*cos5);

      return EE_pos;

  }



  static Eigen::MatrixXd EE_orientation(double theta_1,double theta_2,double theta_3,
  			                double theta_4,double theta_5,double theta_6)
  {
      double r11 = cos(theta_5)*(cos(theta_1)*cos(theta_4) - sin(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) + sin(theta_5)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_2)*cos(theta_3)*sin(theta_1));
      double r12 = sin(theta_6)*(cos(theta_1)*sin(theta_4) + cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) - cos(theta_6)*(sin(theta_5)*(cos(theta_1)*cos(theta_4) - sin(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) - cos(theta_5)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_2)*cos(theta_3)*sin(theta_1)));
      double r13 = cos(theta_6)*(cos(theta_1)*sin(theta_4) + cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) + sin(theta_6)*(sin(theta_5)*(cos(theta_1)*cos(theta_4) - sin(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) - cos(theta_5)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_2)*cos(theta_3)*sin(theta_1)));
      double r21 = cos(theta_5)*(cos(theta_4)*sin(theta_1) + sin(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) - sin(theta_5)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3));
      double r22 = sin(theta_6)*(sin(theta_1)*sin(theta_4) - cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) - cos(theta_6)*(sin(theta_5)*(cos(theta_4)*sin(theta_1) + sin(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) + cos(theta_5)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3)));
      double r23 = cos(theta_6)*(sin(theta_1)*sin(theta_4) - cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) + sin(theta_6)*(sin(theta_5)*(cos(theta_4)*sin(theta_1) + sin(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) + cos(theta_5)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3)));
      double r31 = sin(theta_2 + theta_3)*sin(theta_5) - cos(theta_2 + theta_3)*cos(theta_5)*sin(theta_4);
      double r32 = cos(theta_6)*(sin(theta_2 + theta_3)*cos(theta_5) + cos(theta_2 + theta_3)*sin(theta_4)*sin(theta_5)) + cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_6);
      double r33 = cos(theta_2 + theta_3)*cos(theta_4)*cos(theta_6) - sin(theta_6)*(sin(theta_2 + theta_3)*cos(theta_5) + cos(theta_2 + theta_3)*sin(theta_4)*sin(theta_5));

  //    double beta = asin2(-r31);
  //
  //    Eigen::MatrixXd EE_Orientation(3,1);
  //    
  //    EE_Orientation << 
  //  // roll
  //    atan2(r32/cos(beta),r33/cos(beta)),
  //    // pitch
  //    beta,
  //    // yaw
  //    atan2(r21/cos(beta),r11/cos(beta));

      double alpha = atan2(r21,r11);

      Eigen::MatrixXd EE_Orientation(3,1);
    
      EE_Orientation << 
      // roll
      atan2(sin(alpha)*r13-cos(alpha)*r23,-sin(alpha)*r12+cos(alpha)*r22),
      // pitch
      atan2(-r31,cos(alpha)*r11+sin(alpha)*r21),				// limit pitch direction
      // yaw
      alpha;

      return EE_Orientation;

  };

};

#endif //DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
