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

#include "dynamixel_workbench_controllers/torque_control_icros.h"

TorqueControl::TorqueControl()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 3000000);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  p_gain_ = node_handle_.param<double>("p_gain", 0.003);
  d_gain_ = node_handle_.param<double>("d_gain", 0.00002);

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

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
    dxl_wb_->itemWrite(dxl_id_[index], "Operating_Mode", X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE);
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 1);
  }

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
  initServer();

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
  forwardkinematics_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/EE_pose", 10);

  test_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/test", 10);
}

void TorqueControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("/goal_dynamixel_position", 10, &TorqueControl::goalJointPositionCallback, this);
}

void TorqueControl::initServer()
{
  STOP_server_ = node_handle_.advertiseService("/STOP", &TorqueControl::StopCallback, this);
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

  // ROS_INFO("------------------------------------------------");
  // // ROS_INFO("%lf, %lf", present_position_[0], present_position_[1]); 
  
  // sensor_msgs::JointState test;

  // test.header.stamp = ros::Time::now();

  // for (int i = 0; i < 2; i++)
  // {
  //   if (present_position_[i] - present_position_i[i] != 0)
  //     num_deriv[i] = (present_position_[i] - present_position_i[i]) / time_loop;
    
  //   else num_deriv[i] = 0;

  //   present_position_i[i] = present_position_[i];

  //   test.position.push_back(num_deriv[i]);
  // }
  // ROS_WARN("%lf", time_loop);
  // ROS_WARN("%lf, %lf", num_deriv[0], num_deriv[1]);

  // test_pub_.publish(test);
}

void TorqueControl::controlLoop()
{
  const float tilt_motor_mass = 0.082;
  const float gravity         = 9.8;
  const float link_length     = 0.12409;

  int32_t calc_torque[2] = {0, };

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

  // ROS_INFO("%lf",goal_torque[0]);
  // ROS_INFO("%lf",goal_torque[1]);
  // ROS_INFO("-------------------------------------------");
}

void TorqueControl::ForwardKinematics()
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
  // // EE_position에 대한 조건
  // if (EE_position[1] < -0.03)
  //   stopFlag = true; //조건문을 바꿀 여지가 있을지도?

  // // goal(input) torque에 대한 조건
  // for (int index = 0; index < dxl_cnt_; index++)
  // {
  //   if (goal_torque[index] > 0.1)
  //     stopFlag = true; // 전류값 스케일 조정이 필요할지도?
  // }

  // stopFlag == true이면
  if (stopFlag)
  {
    for (int index = 0; index < dxl_cnt_; index++)
      dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);
    
    // ROS_INFO("EMERGENCY!!!");
    // ROS_INFO("Torque X!!!");
  }
  else
  {
    for (int index = 0; index < dxl_cnt_; index++)
      dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 1);
    
    // ROS_INFO("Torque O!!!");
  }
}

bool TorqueControl::StopCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                 dynamixel_workbench_msgs::JointCommand::Response &res)
{
  if(stopFlag) stopFlag=false;
  else stopFlag = true;

  return stopFlag;
}

void TorqueControl::Test()
{
  

}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "torque_control_icros");
  TorqueControl torque_ctrl;

  ros::Rate loop_rate(250);

//  torque_ctrl.time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
    // // time_loop update
    // torque_ctrl.time_f = ros::Time::now().toSec();
		// torque_ctrl.time_loop = torque_ctrl.time_f - torque_ctrl.time_i;
		// torque_ctrl.time_i = ros::Time::now().toSec();

//    torque_ctrl.controlLoop(); // syncWrite
    torque_ctrl.jointStatePublish(); // [sensor_msgs::JointState] /joint_states
//    torque_ctrl.ForwardKinematics(); // solve ForwardKinematics
//    torque_ctrl.safe_func(); // check FK, effort and KILL at emergency
//    torque_ctrl.Test();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}