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

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define PI 3.14159256359

class DasomCamControl
{
 public:
  DasomCamControl();
  ~DasomCamControl();

  //DasomCam *ds_cam_(image_transport::Publisher& publisher, int cam_num_);
  DasomCam *ds_cam_;

  image_transport::Publisher pub;

  Eigen::VectorXd haptic_pose;
  Eigen::VectorXd gimbal_tf;
  Eigen::VectorXd global_EE_tf;

  void L();
  
 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport it_;

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
  ros::Subscriber gimbal_sub_;

  bool grey = false;
  int grey_button = 0;

  void joystickCallback(const geometry_msgs::Twist &msg);
  void buttonCallback(const omni_msgs::OmniButtonEvent &msg);
  void gimbalCallback(const geometry_msgs::PoseStamped &msg);

};

#endif //DASOM_CAMERA_CONTROL_H_
