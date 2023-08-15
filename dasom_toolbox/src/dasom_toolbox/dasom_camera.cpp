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

#include "../../include/dasom_toolbox/dasom_camera.h"

DasomCam::DasomCam(image_transport::Publisher& publisher, int cam_num_)
: nh_(""), it_(nh_)
{
  cam_pub_ = publisher;

  initCamera(cam_num_);
}

DasomCam::~DasomCam()
{
  
}

void DasomCam::test()
{
  while(ros::ok())
  ROS_INFO("THIS IS DASOM CAMERA!");
}

void DasomCam::initCamera(int cam_num)
{
  std::istringstream video_sourceCmd(std::to_string(cam_num));
  
  int video_source;
  
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source))
  {
    ROS_ERROR("[ DasomCam ]: Failed to load Dasom Camera!");
  }

  cap = cv::VideoCapture(video_source);

  if(!cap.isOpened())
  {
    ROS_ERROR("[ DasomCam ]: Failed to open Dasom Camera!");
  } 
}

void DasomCam::UpdateCamera()
{
  // ROS_INFO("Reading camera frame!");
  cap >> frame;

  if(!frame.empty())
  {
//    ROS_INFO("Converting frame to ROS message!");
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
  ros::spinOnce();
}