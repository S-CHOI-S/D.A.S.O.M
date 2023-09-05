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

#include "../../include/dasom_toolbox/dasom_realsense_d435i.h"

using namespace dasom;

DasomRealSense::DasomRealSense(ros::Publisher& c_pub, ros::Publisher& d_pub)
: nh_("")
{
  ROS_INFO("Dasom D435I Camera Node Start!");

  color_pub = c_pub;
  depth_pub = d_pub;

  // 기본적인 노드 초기화 작업 수행
  rs2::context ctx;
  rs2::device_list dev_list = ctx.query_devices();

  if (dev_list.size() == 0) 
  {
    ROS_ERROR("No RealSense devices detected!");
    ros::shutdown();
  }

  initCamera();
}

DasomRealSense::~DasomRealSense()
{
  color_pub.shutdown();
  depth_pub.shutdown();

  ROS_INFO("Bye DasomRealSense!");
  ros::shutdown();
}

void DasomRealSense::test()
{
  ROS_INFO("Here is Dasom RealSense D435I Toolbox!");
}

void DasomRealSense::initCamera()
{
  cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, RS2_FORMAT_BGR8, 30); // Enable color stream
  cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 30); // Enable depth stream

  // Start streaming
  pipe.start(cfg);

  frames = pipe.wait_for_frames();

  ROS_INFO("Dasom D435I Camera is enabled!");
}

void DasomRealSense::updateCamera()
{
  frames = pipe.wait_for_frames();

  ColorFrame();
  DepthFrame();
}

void DasomRealSense::ColorFrame()
{
  // ROS_INFO("Dasom D435I Camera: get color frame");

  try
  {
    // Get color frame
    rs2::video_frame color_frame = frames.get_color_frame();

    if(color_frame)
    {
      DrawPoint2ColorFrame(color_frame);

      // Create a ROS sensor_msgs::Image
      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "dasom_color_frame";
      msg->height = color_frame.get_height(); // 1080
      msg->width = color_frame.get_width(); // 1920
      msg->encoding = "bgr8";  // BGR 8-bit encoding
      msg->step = color_frame.get_stride_in_bytes();
      msg->data.resize(color_frame.get_height() * color_frame.get_stride_in_bytes());
      std::memcpy(msg->data.data(), color_frame.get_data(), msg->data.size());

      color_pub.publish(msg);
    }

    else ROS_WARN("Color frame is null!");
  }

  catch (const rs2::error &e) 
  {
    std::cerr << "RealSense error: " << e.what() << std::endl;
  }
}

void DasomRealSense::DepthFrame()
{
  // ROS_INFO("Dasom D435I Camera: get depth frame");

  try
  {
    // Get depth frame
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    if(depth_frame)
    {
      Point2Distance(depth_frame, point);
      // DrawPoint2DepthFrame(depth_frame);

      // Create a ROS sensor_msgs::Image
      sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

      msg->header.stamp = ros::Time::now();
      msg->header.frame_id = "dasom_depth_frame";
      msg->height = depth_frame.get_height(); // 480
      msg->width = depth_frame.get_width(); // 848
      msg->encoding = "16UC1";  // BGR 8-bit encoding
      msg->step = depth_frame.get_stride_in_bytes();
      msg->data.resize(depth_frame.get_height() * depth_frame.get_stride_in_bytes());
      std::memcpy(msg->data.data(), depth_frame.get_data(), msg->data.size());

      depth_pub.publish(msg);
    }

    else ROS_WARN("Depth frame is null!");
  }

  catch (const rs2::error &e) 
  {
    std::cerr << "RealSense error: " << e.what() << std::endl;
  }  
}

void DasomRealSense::Point2Distance(rs2::depth_frame frame, Eigen::Vector2d point)
{
  // Get the depth value at a specific pixel (x, y)
  distance_point[0] = point[0];
  distance_point[1] = point[1];

  // Get depth in meters
  float depth_meters = frame.get_distance(distance_point[0], distance_point[1]); 

  // Print the distance in meters
  std::cout << "Distance at (" << distance_point[0] << ", " << distance_point[1] << "): " << depth_meters << " meters" << std::endl;
}

void DasomRealSense::DrawPoint2ColorFrame(rs2::video_frame frame)
{
  double distance_x;
  double distance_y;
  
  // Convert rs2::video_frame to OpenCV Mat
  cv::Mat color_mat(cv::Size(frame.get_width(), frame.get_height()), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

  distance_x = map(distance_point[0], 0, 848, 0, 1920);
  distance_y = map(distance_point[1], 0, 480, 0, 1080);

  cv::circle(color_mat, cv::Point(distance_x, distance_y), 10, red, -1);

  cv::putText(color_mat, "distance", cv::Point(distance_x - 70, distance_y + 40), 3, 1, red, 2, 8);
}

void DasomRealSense::DrawPoint2DepthFrame(rs2::depth_frame frame)
{
  // Convert rs2::video_frame to OpenCV Mat
  cv::Mat color_mat(cv::Size(frame.get_width(), frame.get_height()), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

  cv::circle(color_mat, cv::Point(distance_point[0], distance_point[1]), 10, red, -1);

  cv::putText(color_mat, "distance", cv::Point(distance_point[0] - 70, distance_point[1] + 40), 3, 1, red, 2, 8);
}