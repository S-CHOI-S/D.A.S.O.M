#ifndef DASOM_CAMERA_CONTROL_H_
#define DASOM_CAMERA_CONTROL_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <kdl/chain.hpp>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>
#include <dynamixel_workbench_msgs/EECommand.h>
#include "dasom_controllers/movingFlag.h"
#include "dasom_controllers/admittanceTest.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include "tf/transform_datatypes.h"
#include "omni_msgs/OmniButtonEvent.h"

#include <dasom_toolbox/dasom_workbench.h>
#include <dasom_toolbox/dasom_camera.h>
#include <dasom_toolbox/dasom_tf2.h>



#define PI 3.14159256359

class DasomCamControl
{
 public:
  DasomCamControl();
  ~DasomCamControl();

  DasomCam *ds_cam(image_transport::Publisher& publisher, int cam_num_);

  Eigen::VectorXd haptic_pose;

  
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
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joystick_sub_;
  ros::Subscriber button_sub_;

  void joystickCallback(const geometry_msgs::Twist &msg);
  void buttonCallback(const omni_msgs::OmniButtonEvent &msg);

};

#endif //DASOM_CAMERA_CONTROL_H_
