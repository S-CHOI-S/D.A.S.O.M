/*******************************************************************************
* D.A.S.O.M
*
* Department of Aerial Manipulator System for Object Manipulation
*
*     https://github.com/S-CHOI-S/D.A.S.O.M.git
*
* Mobile Robotics Lab. (MRL)
* 	  @ Seoul National University of Science and Technology
*
*	  https://mrl.seoultech.ac.kr/index.do
*
*******************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#ifndef DASOM_CAMERA_H_
#define DASOM_CAMERA_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define black cv::Scalar::all(0)
#define white cv::Scalar::all(255)
#define red cv::Scalar(255,0,0)
#define green cv::Scalar(0,255,0)
#define blue cv::Scalar(0,0,255)

class DasomCam
{
 public:
  DasomCam(image_transport::Publisher& publisher, int cam_num_);
  ~DasomCam();

  image_transport::Publisher cam_pub_;

  cv::Mat frame;

  bool gimbalcommand_safe = false;
  
  void UpdateCameraCommand(Eigen::Vector3d core);
  void DrawGimbalCross(Eigen::Vector3d gimbal, cv::Scalar color);
  void UpdateCameraGimbal(Eigen::Vector3d core, Eigen::Vector3d gimbal);
  void UpdateCameraGimbalCommand(Eigen::Vector3d core, Eigen::Vector3d gimbal);
  void test();

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  cv::VideoCapture cap;
  sensor_msgs::ImagePtr msg;

  int i = 0;

  void initCamera(int cam_num);

};

#endif /*DASOM_CAMERA_H_*/
