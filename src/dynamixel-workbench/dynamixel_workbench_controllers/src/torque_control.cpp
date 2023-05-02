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

#include "dynamixel_workbench_controllers/torque_control.h"

TorqueControl::TorqueControl()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 3000000);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  p_gain_ = node_handle_.param<float>("p_gain", 0.003);
  d_gain_ = node_handle_.param<float>("d_gain", 0.00002);

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
  {
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
    dxl_wb_->itemWrite(dxl_id_[index], "Operating_Mode", X_SERIES_CURRENT_CONTROL_MODE);
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 1);
  }

  dxl_wb_->addSyncWrite("Goal_Current");
  dxl_wb_->addSyncRead("Present_Position");

  initPublisher();
  initSubscriber();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
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
  printf("              D.A.S.O.M controller; torque control method              \n");
  printf("                -This method supports XM430-W350 only-                 \n");
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
  forwardkinematics_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/EE_pose",10);
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

void TorqueControl::controlLoop()
{
  const float tilt_motor_mass = 0.082;
  const float gravity         = 9.8;
  const float link_length     = 0.12409;

  int32_t calc_torque[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
  {
    calc_torque[index] = dxl_wb_->convertTorque2Value(dxl_id_[index], goal_torque[index]);
  }

  dxl_wb_->syncWrite("Goal_Current", calc_torque);
}

void TorqueControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (int index = 0; index < dxl_cnt_; index++)
    goal_torque[index] = msg->effort.at(index);
}

void TorqueControl::ForwardKinematics()
// 여기는 사용할 때 6DOF에 맞게 바꿔야 함!
{
  geometry_msgs::Twist EndEffector;

  EE_position = EE_pos(present_position_[0], present_position_[1]);

  EndEffector.linear.x = EE_position[0];
  EndEffector.linear.y = EE_position[1];

  forwardkinematics_pub_.publish(EndEffector);
}

void TorqueControl::safe_func()
// stopFlag의 init 값은 flase
{
  // EE_position에 대한 조건
  if (EE_position[0] < -0.03)
    stopFlag = true; //조건문을 바꿀 여지가 있을지도?

  // goal(input) torque에 대한 조건
  for (int index = 0; index < dxl_cnt_; index++)
  {
    if (goal_torque[index] > 0.1)
      stopFlag = true; // 전류값 스케일 조정이 필요할지도?
  }

  // stopFlag == true이면
  if (stopFlag)
  {
    for (int index = 0; index < dxl_cnt_; index++)
      dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
    
    ROS_INFO("EMERGENCY!!!");
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "torque_control");
  TorqueControl torque_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    torque_ctrl.controlLoop(); // syncWrite
    torque_ctrl.jointStatePublish(); // [sensor_msgs::JointState] /joint_states
    torque_ctrl.ForwardKinematics(); // solve ForwardKinematics
    torque_ctrl.safe_func(); // check FK, effort and KILL at emergency
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

// FK 넣고 (O)
// 전류값 limit