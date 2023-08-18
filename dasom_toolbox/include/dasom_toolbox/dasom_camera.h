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

class DasomCam
{
 public:
  DasomCam(image_transport::Publisher& publisher, int cam_num_);
  ~DasomCam();

  image_transport::Publisher cam_pub_;
  
  void UpdateCamera(double core_x, double core_y, double core_z);
  void test();

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  cv::VideoCapture cap;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  void initCamera(int cam_num);

};

#endif /*DASOM_CAMERA_H_*/
