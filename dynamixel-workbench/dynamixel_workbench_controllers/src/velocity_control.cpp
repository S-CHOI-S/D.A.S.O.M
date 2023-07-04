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

#include "dynamixel_workbench_controllers/velocity_control.h"

VelocityControl::VelocityControl()
    :node_handle_(""),
     dxl_cnt_(2)
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  uint32_t velocity_p_gain	= node_handle_.param<int>("velocity_p_gain", 800);
  uint32_t velocity_i_gain	= node_handle_.param<int>("velocity_i_gain", 0);

  wheel_separation_ = node_handle_.param<float>("wheel_separation", 0.160);
  wheel_radius_     = node_handle_.param<float>("wheel_radius", 0.033);

  dxl_id_[0] = node_handle_.param<int>("left_wheel", 1);
  dxl_id_[1] = node_handle_.param<int>("right_wheel", 2);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  for (int index = 0; index < dxl_cnt_; index++)
  {
    uint16_t get_model_number;
    if (dxl_wb_->ping(dxl_id_[index], &get_model_number) != true)
    {
      ROS_ERROR("Not found Motors, Please check id and baud rate");

      ros::shutdown();
      return;
    }
  }

  initMsg();

  // Set Reverse Mode to Right Motor(ID : 2)
  //dxl_wb_->itemWrite(dxl_id_[1], "Drive_Mode", 1);

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dxl_wb_->wheelMode(dxl_id_[index], profile_velocity, profile_acceleration);
      dxl_wb_->itemWrite(dxl_id_[index], "Velocity_P_Gain", velocity_p_gain);
      dxl_wb_->itemWrite(dxl_id_[index], "Velocity_I_Gain", velocity_i_gain); 
  }


  dxl_wb_->addSyncWrite("Goal_Velocity");

  dxl_wb_->addSyncRead("Present_Velocity"); // ADD
  dxl_wb_->addSyncRead("Present_Position"); // ADDD


  //--------------토크껐음------------//
  //for (int index = 0; index < 2; index++)
  //  dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  initPublisher();
  initSubscriber();
  initServer();
}

VelocityControl::~VelocityControl()
{
  for (int index = 0; index < 2; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void VelocityControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("        dynamixel_workbench controller; velocity control example       \n");
  printf("              -This example supports MX2.0 and X Series-               \n");
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

void VelocityControl::initPublisher()
{
  // dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
  dasom_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/dasom/joint_states", 10);
}

void VelocityControl::initSubscriber()
{
  cmd_vel_sub_ = node_handle_.subscribe("/dasom/goal_dynamixel_position", 10, &VelocityControl::commandVelocityCallback, this);
}

void VelocityControl::initServer()
{
  // wheel_command_server_ = node_handle_.advertiseService("wheel_command", &VelocityControl::wheelCommandMsgCallback, this);
}

// void VelocityControl::dynamixelStatePublish()
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

void VelocityControl::controlLoop()
{
  // dynamixelStatePublish();
  sensor_msgs::JointState dynamixel_; // ADD
  // sensor_msgs::JointState dynamixel_; // ADD
  double conv_position[dxl_cnt_] = {0, };
//  int32_t* raw_velocity = dxl_wb_->syncRead("Present_Velocity");
//  int32_t* raw_position = dxl_wb_->syncRead("Present_Position");


  int16_t present_current[dxl_cnt_] = {0, };  
  int16_t present_position[dxl_cnt_] = {0, };  
  int16_t present_velocity[dxl_cnt_] = {0, };  

  for (int index = 0; index < dxl_cnt_; index++)
  {
    present_current[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Current");
    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
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


    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index],present_velocity[index]));
    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index],present_position[index]));
    dynamixel_.effort.push_back(dxl_wb_->convertValue2Torque(dxl_id_[index], present_current[index]));
  }
  dasom_joint_states_pub_.publish(dynamixel_); // ADD
}

// bool VelocityControl::wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
//                                               dynamixel_workbench_msgs::WheelCommand::Response &res)
// {
//   static int32_t goal_velocity[2] = {0, 0};

//   goal_velocity[0] = dxl_wb_->convertVelocity2Value(dxl_id_[0], req.left_vel);
//   goal_velocity[1] = dxl_wb_->convertVelocity2Value(dxl_id_[1], (-1) * req.right_vel);

//   bool ret = dxl_wb_->syncWrite("Goal_Velocity", goal_velocity);

//   res.result = ret;
// }

void VelocityControl::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  bool dxl_comm_result = false;

  float wheel_velocity[2] = {0.0, 0.0};
  int32_t dynamixel_velocity[2] = {0, 0};

  double lin_vel = msg->linear.x;
  double ang_vel = msg->angular.z;

  const float RPM_OF_DXL = 0.229;
   const float VELOCITY_CONSTANT_VALUE = 1 / (wheel_radius_ * RPM_OF_DXL * 0.10472);

  // wheel_velocity[0]  = lin_vel - (ang_vel * wheel_separation_ / 2);
  // wheel_velocity[1]  = lin_vel + (ang_vel * wheel_separation_ / 2);

  // dynamixel_velocity[0] = wheel_velocity[0] * VELOCITY_CONSTANT_VALUE;
  // dynamixel_velocity[1] = wheel_velocity[1] * VELOCITY_CONSTANT_VALUE;

  dynamixel_velocity[0]=lin_vel* VELOCITY_CONSTANT_VALUE / 26;
  dynamixel_velocity[1]=ang_vel* VELOCITY_CONSTANT_VALUE / 26;

  dxl_wb_->syncWrite("Goal_Velocity", dynamixel_velocity);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "velocity_control");
  VelocityControl vel_ctrl;

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    vel_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}