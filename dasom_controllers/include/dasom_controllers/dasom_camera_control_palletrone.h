#ifndef DASOM_CAMERA_CONTROL_PALLETRONE_H_
#define DASOM_CAMERA_CONTROL_PALLETRONE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <dasom_toolbox/dasom_camera.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace dasom;

class DasomCamPalletrone : public dasom::DasomCam
{
 public:
  DasomCamPalletrone();
  ~DasomCamPalletrone();

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void update();
  
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
  ** ROS Publishers
  *****************************************************************************/
  image_transport::Publisher pub;

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joystick_sub_;
  ros::Subscriber button_sub_;
  ros::Subscriber gimbal_sub_;

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For haptic control
  Eigen::Vector3d haptic_position;

  // For haptic button
  bool grey = false;
  int grey_button = 0;

  // For gimbaling
  Eigen::VectorXd global_EE_tf;
  Eigen::Vector3d gimbal_position_tf;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void gimbalCallback(const geometry_msgs::PoseStamped &msg);
  void joystickCallback(const geometry_msgs::Twist &msg);
  void buttonCallback(const omni_msgs::OmniButtonEvent &msg);
};

#endif //DASOM_CAMERA_CONTROL_PALLETRONE_H_
