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

using namespace dasom;

DasomCam::DasomCam(image_transport::Publisher& publisher, int cam_num_)
: nh_(""), it_(nh_)
{
  cam_pub_ = publisher;

  initCamera(cam_num_);
}

DasomCam::~DasomCam()
{
  cam_pub_.shutdown();

  ROS_INFO("Bye DasomCam!");
  ros::shutdown();
}

void DasomCam::test()
{
  ROS_INFO("THIS IS DASOM CAMERA!");
}

void DasomCam::reInitializePublisher(image_transport::Publisher& publisher)
{
  cam_pub_ = publisher;
}

void DasomCam::initCamera(int cam_num)
{
  std::istringstream video_sourceCmd(std::to_string(cam_num));
  
  int video_source;
  
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source))
  {
    ROS_ERROR("[ DasomCam ]: Failed to load Dasom Camera #%d!", cam_num);
  }

  cap = cv::VideoCapture(video_source);

  if(!cap.isOpened())
  {
    ROS_ERROR("[ DasomCam ]: Failed to open Dasom Camera #%d!", cam_num);
  }
  else
  {
    ROS_WARN("[ DasomCam ]: Successed to open Dasom Camera #%d!", cam_num);
  }
}

void DasomCam::UpdateCameraCommand(Eigen::Vector3d core)
{
  // ROS_INFO("Reading camera frame!");
  cap >> frame;

  gimbalcommand_safe = false; // ROS_WARN("%lf, %lf, %lf", core[0], core[1], core[2]);
  
  // frame = flipCamera(frame);

  frame = rotateCamera(frame);
  cv::flip(frame, frame, -1);

  DetectLightBulb();

  frame = drawCoordinate(frame);

  frame = drawHapticJoystick(frame, core);

  // cv::imshow("D.A.S.O.M End_Effector", frame);

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
  // ros::spinOnce();
}

cv::Mat DasomCam::drawHapticJoystick(cv::Mat frame, Eigen::Vector3d core)
{
  // circle(frame, core, radius, color, thickness, line type, shift);
  circle(frame, cv::Point(240, 320), 180 + core[2], blue, 3, 4, 0);
  
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(220, 320), cv::Point(260, 320), cv::Scalar::all(255), 3, 4, 0);
  line(frame, cv::Point(240, 300), cv::Point(240, 340), cv::Scalar::all(255), 3, 4, 0);

  return frame;
}

void DasomCam::DrawGimbalCross(Eigen::Vector3d gimbal, cv::Scalar color)
{
  // line(frame, point1, point2, color, thickness, line type, shift);
  line(frame, cv::Point(220, 320), cv::Point(260, 320), color, 3, 4, 0);
  line(frame, cv::Point(240, 300), cv::Point(240, 340), color, 3, 4, 0);

  // putText(frame, string, Point(x,y), font face, font scale, color, thickness, line type, bottom left origin);
  putText(frame, "gimbal", cv::Point(203, 360), 3, 0.7, color, 1, 8);
}

void DasomCam::UpdateCameraGimbal(Eigen::Vector3d core, Eigen::Vector3d gimbal)
{
  cap >> frame;

  i = 0;
  ROS_WARN("i = %d", i);

  frame = rotateCamera(frame); 
  cv::flip(frame, frame, -1);

  DetectLightBulb();

  frame = drawCoordinate(frame);

  frame = drawHapticJoystick(frame, core);

  DrawGimbalCross(gimbal, white);

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

  frame = rotateCamera(frame);
  cv::flip(frame, frame, -1);
  
  DetectLightBulb();

  frame = drawCoordinate(frame);

  frame = drawHapticJoystick(frame, core);

  // draw moving gimbal
  line(frame, cv::Point(220-core[1], 230+core[0]), cv::Point(260-core[1], 230+core[0]), blue, 3, 4, 0);
  line(frame, cv::Point(240-core[1], 210+core[0]), cv::Point(240-core[1], 250+core[0]), blue, 3, 4, 0);

  if(abs(core[1] - gimbal[1]) < 3 && abs(core[0] - gimbal[0]) < 3)
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

void DasomCam::UpdateCameraPalletrone()
{
  // ROS_INFO("Reading camera frame!");
  cap >> frame;

  frame = flipCamera(frame);

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    cam_pub_.publish(msg);

    cv::waitKey(1);
  }
}

void DasomCam::DetectLightBulb()
{
  cv::Mat frame_hsv;
  cv::Mat frame_erode;
  cv::Mat frame_dilate;

  cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

  cv::Mat lightbulb_mask, lightbulb_frame;

  cv::Scalar lower_lightbulb = cv::Scalar(15,30,100);
  cv::Scalar upper_lightbulb = cv::Scalar(75,255,255);

  cv::inRange(frame_hsv, lower_lightbulb, upper_lightbulb, lightbulb_mask);
  cv::bitwise_and(frame, frame, lightbulb_frame, lightbulb_mask);

  cv::erode(lightbulb_mask, frame_erode, cv::Mat(), cv::Point(-1,-1), 5);
  cv::dilate(frame_erode, frame_dilate, cv::Mat(), cv::Point(-1,-1), 2);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(frame_dilate, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  if (!contours.empty()) 
  {
    for (size_t i = 0; i < contours.size(); ++i) 
    {
      double area = cv::contourArea(contours[i]);
      if (area > 3000) 
      {
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);
        cv::Point2f boxPoints[4];
        rect.points(boxPoints);
        for (int j = 0; j < 4; ++j) 
        {
          cv::line(frame, boxPoints[j], boxPoints[(j + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }

        // 중심 좌표를 이용하여 텍스트 위치 계산
        cv::Point2f center = rect.center;
        cv::putText(frame, "light bulb", cv::Point(center.x - 40, center.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 2);
      }
    }
  }

  ROS_WARN("Detect!");
}

cv::Mat DasomCam::flipCamera(cv::Mat frame)
{
  cv::Mat flipped_frame;

  cv::flip(frame, flipped_frame, 1);

  return flipped_frame;
}

cv::Mat DasomCam::rotateCamera(cv::Mat frame)
{
  cv::Mat rotated_frame;

  cv::rotate(frame, rotated_frame, cv::ROTATE_90_COUNTERCLOCKWISE);

  return rotated_frame;
}

cv::Mat DasomCam::drawCoordinate(cv::Mat frame)
{
  arrowedLine(frame, cv::Point(20, 470), cv::Point(50, 445), blue, 3, cv::LINE_AA);
  arrowedLine(frame, cv::Point(20, 470), cv::Point(60, 470), green, 3);
  arrowedLine(frame, cv::Point(20, 470), cv::Point(20, 430), red, 3);

  return frame;
}
