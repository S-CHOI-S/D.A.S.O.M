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

void DasomCam::UpdateCameraCommand(Eigen::Vector3d core)
{
  // ROS_INFO("Reading camera frame!");
  cap >> frame;

  gimbalcommand_safe = false;

  // circle(frame, core, radius, color, thickness, line type, shift);
  circle(frame, cv::Point(250 - core[0], 250 - core[1]), 200 - core[2], cv::Scalar(255,0,0), 3, 4, 0);
  
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(230 - core[0], 250 - core[1]), cv::Point(270 - core[0], 250 - core[1]), cv::Scalar::all(255), 3, 4, 0);
  line(frame, cv::Point(250 - core[0], 230 - core[1]), cv::Point(250 - core[0], 270 - core[1]), cv::Scalar::all(255), 3, 4, 0);

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
  ros::spinOnce();
}

void DasomCam::DrawGimbalCross(Eigen::Vector3d gimbal, cv::Scalar color)
{
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(230 - gimbal[0], 250 - gimbal[1]), cv::Point(270 - gimbal[0], 250 - gimbal[1]), color, 3, 4, 0);
  line(frame, cv::Point(250 - gimbal[0], 230 - gimbal[1]), cv::Point(250 - gimbal[0], 270 - gimbal[1]), color, 3, 4, 0);

  // putText(frame, string, Point(x,y), font face, font scale, color, thickness, line type, bottom left origin);
  putText(frame, "gimbal", cv::Point(210 - gimbal[0], 290 - gimbal[1]), 3, 0.7, color, 1, 8);
}

void DasomCam::UpdateCameraGimbal(Eigen::Vector3d core, Eigen::Vector3d gimbal)
{
  cap >> frame;

  i = 0;
  ROS_WARN("i = %d", i);

  DrawGimbalCross(gimbal, white);

  // circle(frame, core, radius, color, thickness, line type, shift);
  circle(frame, cv::Point(250 - core[0], 250 - core[1]), 200 - core[2], cv::Scalar(255,0,0), 3, 4, 0);
  
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(230 - core[0], 250 - core[1]), cv::Point(270 - core[0], 250 - core[1]), cv::Scalar::all(255), 3, 4, 0);
  line(frame, cv::Point(250 - core[0], 230 - core[1]), cv::Point(250 - core[0], 270 - core[1]), cv::Scalar::all(255), 3, 4, 0);

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
  ros::spinOnce();
}

void DasomCam::UpdateCameraGimbalCommand(Eigen::Vector3d core, Eigen::Vector3d gimbal)
{
  cap >> frame;
  ROS_ERROR("%lf, %lf", core[0] - gimbal[0], core[1] - gimbal[1]);

  // circle(frame, core, radius, color, thickness, line type, shift);
  circle(frame, cv::Point(250 - core[0], 250 - core[1]), 200 - core[2], cv::Scalar(255,0,0), 3, 4, 0);
  
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(230 - core[0], 250 - core[1]), cv::Point(270 - core[0], 250 - core[1]), cv::Scalar::all(255), 3, 4, 0);
  line(frame, cv::Point(250 - core[0], 230 - core[1]), cv::Point(250 - core[0], 270 - core[1]), cv::Scalar::all(255), 3, 4, 0);

  if(abs(core[0] - gimbal[0]) < 1.5 && abs(core[1] - gimbal[1]) < 1.5)
  {
    ROS_INFO("Start Gimbaling + Command mode!");
     
    DrawGimbalCross(gimbal, green);
    
    i++; 
    ROS_WARN("i = %d", i);
    
    if(i >= 50) gimbalcommand_safe = true;
  }
  else  
  {
    DrawGimbalCross(gimbal, white);
    i = 0;
    gimbalcommand_safe = false;
  }

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
  ros::spinOnce();
}