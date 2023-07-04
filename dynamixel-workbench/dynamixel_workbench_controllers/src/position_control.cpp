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

#include "dynamixel_workbench_controllers/position_control.h"
#include "geometry_msgs/Twist.h"
#include <math.h>


PositionControl::PositionControl()
    :node_handle_("")
{
  robot_name_   = node_handle_.param<std::string>("robot_name", "dasom");

  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  uint32_t position_p_gain	= node_handle_.param<int>("position_p_gain", 800);
  uint32_t position_i_gain	= node_handle_.param<int>("position_i_gain", 0);
  uint32_t position_d_gain	= node_handle_.param<int>("position_d_gain", 0);

  uint32_t feedforward_1st_gain = node_handle_.param<int>("feedforward_1st_gain", 0);
  uint32_t feedforward_2nd_gain = node_handle_.param<int>("feedforward_2nd_gain", 0);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Position");
  
  dxl_wb_->addSyncRead("Present_Velocity"); // ADDD
  dxl_wb_->addSyncRead("Present_Position"); // ADD
  // dxl_wb_->addSyncRead("Present_Current"); // ADDD ASDF 
  for (int index = 0; index < dxl_cnt_; index++){
      dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);
      dxl_wb_->itemWrite(dxl_id_[index], "Position_P_Gain", position_p_gain);
      dxl_wb_->itemWrite(dxl_id_[index], "Position_I_Gain", position_i_gain);
      dxl_wb_->itemWrite(dxl_id_[index], "Position_D_Gain", position_d_gain);
      dxl_wb_->itemWrite(dxl_id_[index], "Feedforward_1st_Gain", feedforward_1st_gain);
      dxl_wb_->itemWrite(dxl_id_[index], "Feedforward_2nd_Gain", feedforward_2nd_gain);
    }

  initPublisher();
  initSubscriber();
  initServer();
}

PositionControl::~PositionControl()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void PositionControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  D.A.S.O.M controller; position control             \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void PositionControl::initPublisher()
{
  // dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("/dynamixel_state", 10);
  // joint_states_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DasomDynamixel>(robot_name_ + "/joint_states", 10);
  dasom_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/joint_states", 10);
  TestPub = node_handle_.advertise<geometry_msgs::Twist>("/Test_Pub", 10);
}

void PositionControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("/dasom/goal_dynamixel_position", 10, &PositionControl::goalJointPositionCallback, this);
}

void PositionControl::initServer()
{
  joint_command_server_ = node_handle_.advertiseService("/joint_command", &PositionControl::jointCommandMsgCallback, this);
}

// void PositionControl::dynamixelStatePublish()
// {
//   dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
//   dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

//   for (int index = 0; index < dxl_cnt_; index++)
//   {
//     dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
//     dynamixel_state[index].id                  = dxl_id_[index];
//     dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
//     dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
//     dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
//     dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
//     dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
//     dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

//     dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
//   }
//   dynamixel_state_list_pub_.publish(dynamixel_state_list);
// }

// void PositionControl::jointStatePublish()
// {
//   int32_t present_position[dxl_cnt_] = {0, };

//   for (int index = 0; index < dxl_cnt_; index++)
//     present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

//   int32_t present_velocity[dxl_cnt_] = {0, };

//   for (int index = 0; index < dxl_cnt_; index++)
//     present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

//   sensor_msgs::JointState dynamixel_;
//   dynamixel_.header.stamp = ros::Time::now();

//   for (int index = 0; index < dxl_cnt_; index++)
//   {
//     std::stringstream id_num;
//     id_num << "id_" << (int)(dxl_id_[index]);

//     dynamixel_.name.push_back(id_num.str());

//     dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
//     dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
//   }
//   joint_states_pub_.publish(dynamixel_);
// }

void PositionControl::controlLoop()
{
  // dynamixelStatePublish();
  // jointStatePublish();
  // geometry_msgs::Twist msg; // delete
  dynamixel_workbench_msgs::DasomDynamixel dynamixel_; // ADD
  // sensor_msgs::JointState dynamixel_; // ADD
  double conv_position[dxl_cnt_] = {0, };
//  int32_t* raw_position = dxl_wb_->syncRead("Present_Position");
//  int32_t* raw_current = dxl_wb_->syncRead("Present_Current");  //ADDD ASDF
  int32_t* raw_velocity = dxl_wb_->syncRead("Present_Velocity");  //ADDD

  double raw_current[dxl_cnt_] = {0 , };
  double raw_position[dxl_cnt_] = {0 , };

   for (int index = 0; index < dxl_cnt_; index++) //ADDD 
    {  
    raw_current[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Current");
    raw_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    }

  // for (int index = 0; index < dxl_cnt_; index++) // delete
  // {
  //   conv_position[index] = dxl_wb_->convertValue2Radian(dxl_id_[index],raw_position[index]);


  //   msg.linear.x = conv_position[0];
  //   msg.linear.y = conv_position[1];
  //   msg.linear.z = conv_position[2];
  //   msg.angular.x = conv_position[3];
  // }

  // ROS_INFO("syncRead : %lf", conv_position[0]);
  // ROS_WARN("syncRead : %lf", conv_position[1]);
  // ROS_FATAL("syncRead : %lf", conv_position[2]);

  dynamixel_.header.stamp = ros::Time::now(); // ADD

  for (int index = 0; index < dxl_cnt_; index++) // ADD
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(dxl_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index],raw_position[index]));
    dynamixel_.effort.push_back(dxl_wb_->convertValue2Torque(dxl_id_[index], raw_current[index])); // ADDD ASDF
    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], raw_velocity[index])); // ADDD
    // velocity
    // effort
  }
/*
  if (!(-M_PI < dynamixel_.position[0] && dynamixel_.position[0] < M_PI))
  {
    // ROS_ERROR("Error[001] : Joint1 is limited!\n");
    dynamixel_.flag.push_back(1);
    flag1 = false;
  }
  else
  {
    dynamixel_.flag.push_back(0);
    flag1 = true;
  }

  if (!(-M_PI_2 < dynamixel_.position[1] && dynamixel_.position[1] < 1.53))
  {
    // ROS_ERROR("Error[002] : Joint2 is limited!\n");
    dynamixel_.flag.push_back(1);
  }
  else
    dynamixel_.flag.push_back(0);

  if (!(-M_PI_2 < dynamixel_.position[2] && dynamixel_.position[2] < 1.53))
  {
    // ROS_ERROR("Error[003] : Joint3 is limited!\n");
    dynamixel_.flag.push_back(1);
  }
  else dynamixel_.flag.push_back(0);
*/
  // TestPub.publish(msg); // delete
  dasom_joint_states_pub_.publish(dynamixel_); // ADD
}

bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  int32_t goal_position = 0;
  int32_t present_position = 0;

  if (req.unit == "rad")
  {
    goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
  }
  else if (req.unit == "raw")
  {
    goal_position = req.goal_position;
  }
  else
  {
    goal_position = req.goal_position;
  }

  bool ret = dxl_wb_->goalPosition(req.id, goal_position);

  res.result = ret;
}


void PositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double goal_position[dxl_cnt_] = {0.0, };

  for (int index = 0; index < dxl_cnt_; index++)
    goal_position[index] = msg->position.at(index);
  
  int32_t goal_dxl_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
  {
    goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
  }

/*  if(flag2 == true)
  {
    ROS_INFO("here!");
    dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);
  }
  else ROS_INFO("there||")
  */

    dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);

 // ROS_INFO("%lf, %lf, %lf",goal_dxl_position[0],goal_dxl_position[1],goal_dxl_position[2]);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "position_control");
  PositionControl pos_ctrl;


  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}