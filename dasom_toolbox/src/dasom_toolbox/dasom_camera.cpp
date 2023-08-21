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
  ROS_WARN("HHEERREE!");
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

void DasomCam::UpdateCamera(double core_x, double core_y, double core_z)
{
  // ROS_INFO("Reading camera frame!");
  cap >> frame;

  // circle(frame, core, radius, color, thickness, line type, shift);
  circle(frame, cv::Point(250 - core_x, 250 - core_y), 200 - core_z, cv::Scalar(255,0,0), 3, 4, 0);
  
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(230 - core_x, 250 - core_y), cv::Point(270 - core_x, 250 - core_y), cv::Scalar::all(255), 3, 4, 0);
  line(frame, cv::Point(250 - core_x, 230 - core_y), cv::Point(250 - core_x, 270 - core_y), cv::Scalar::all(255), 3, 4, 0);

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
  ros::spinOnce();
}

void DasomCam::DrawGimbalCross(double core_x, double core_y, double core_z)
{
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(230 - core_x, 250 - core_y), cv::Point(270 - core_x, 250 - core_y), cv::Scalar::all(255), 3, 4, 0);
  line(frame, cv::Point(250 - core_x, 230 - core_y), cv::Point(250 - core_x, 270 - core_y), cv::Scalar::all(255), 3, 4, 0);

  // putText(frame, string, Point(x,y), font face, font scale, color, thickness, line type, bottom left origin);
  putText(frame, "gimbal", cv::Point(210 - core_x, 290 - core_y), 3,0.7,cv::Scalar::all(255),1,8);
}