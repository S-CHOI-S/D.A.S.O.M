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

#include "dynamixel_workbench_controllers/torque_ctrl_6DOF.h"

TorqueControl::TorqueControl()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 3000000);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  double position_p_gain	= node_handle_.param<double>("position_p_gain", 800);
  double position_i_gain	= node_handle_.param<double>("position_i_gain", 0);
  double position_d_gain	= node_handle_.param<double>("position_d_gain", 0);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  // for (int index = 0; index < dxl_cnt_; index++)
  // {
  //   dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
  //   dxl_wb_->itemWrite(dxl_id_[index], "Operating_Mode", X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE);
  //   dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 1);
  // }

  dxl_wb_->addSyncWrite("Goal_Position");
  dxl_wb_->addSyncRead("Present_Position");


  for (int index = 0; index < dxl_cnt_; index++)
  {
    dxl_wb_->itemWrite(dxl_id_[index], "Position_P_Gain", position_p_gain);
    dxl_wb_->itemWrite(dxl_id_[index], "Position_I_Gain", position_i_gain);
    dxl_wb_->itemWrite(dxl_id_[index], "Position_D_Gain", position_d_gain);
  }

  initPublisher();
  initSubscriber();

}

TorqueControl::~TorqueControl()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void TorqueControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("      D.A.S.O.M controller; current based position control method      \n");
  printf("           -This method supports XM430-W350, XH540-W270 only-          \n");
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

void TorqueControl::initPublisher()
{
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  forwardkinematics_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/EE_pose", 10);

  test_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/test", 10);
}

void TorqueControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("/goal_dynamixel_position", 10, &TorqueControl::goalJointPositionCallback, this);
}

void TorqueControl::jointStatePublish()
// publish present joint state
// sensor_msgs::JointState /joint_states
{
  int32_t present_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

  int32_t present_velocity[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

  int16_t present_current[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_current[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Current");

  sensor_msgs::JointState dynamixel_;
  dynamixel_.header.stamp = ros::Time::now();

  for (int index = 0; index < dxl_cnt_; index++)
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(dxl_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
    dynamixel_.effort.push_back(dxl_wb_->convertValue2Torque(dxl_id_[index], present_current[index]));

    present_position_[index] = dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]);
  }
  joint_states_pub_.publish(dynamixel_);
}

void TorqueControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (int index = 0; index < dxl_cnt_; index++)
    goal_position[index] = msg->position.at(index);


  for (int index = 0; index < dxl_cnt_; index++)
  {
    goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
  }

  dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);
}

void TorqueControl::Test()
// 필요할 때 잠깐 쓸 수 있게 만들어 놓음(현재 사용하는 용도 없음)
{
  

}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "torque_ctrl_6DOF");
  TorqueControl torque_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    torque_ctrl.jointStatePublish(); // [sensor_msgs::JointState] /joint_states
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}