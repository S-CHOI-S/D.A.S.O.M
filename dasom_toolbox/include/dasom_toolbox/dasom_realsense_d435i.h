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

#ifndef DASOM_REALSENSE_D435I_H_
#define DASOM_REALSENSE_D435I_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#define black cv::Scalar::all(0)
#define white cv::Scalar::all(255)
#define blue cv::Scalar(255,0,0)
#define green cv::Scalar(0,255,0)
#define red cv::Scalar(0,0,255)

namespace dasom
{
  class DasomRealSense
  {
  public:
    DasomRealSense(ros::Publisher& c_pub, ros::Publisher& d_pub);
    ~DasomRealSense();

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
    void test();
    void updateCamera();

  private:
    /*****************************************************************************
    ** ROS NodeHandle
    *****************************************************************************/
    ros::NodeHandle nh_;

    /*****************************************************************************
    ** ROS Publishers
    *****************************************************************************/
    ros::Publisher color_pub;
    ros::Publisher depth_pub;
    
    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    // For camera
    // Create a RealSense pipeline and configuration
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    
    // For distance
    Eigen::Vector2d point;
    Eigen::Vector2d distance_point;

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
    void initCamera();
    void ColorFrame();
    void DepthFrame();
    void Point2Distance(rs2::depth_frame frame, Eigen::Vector2d point);
    void DrawPoint2ColorFrame(rs2::video_frame frame);
    void DrawPoint2DepthFrame(rs2::depth_frame frame);

    double map(double input_data, double input_min, double input_max, double output_min, double output_max)
    {
      return (input_data - input_min) * (output_max - output_min) / (input_max - input_min) + output_min;
    }
  };
} // namespace DASOM

#endif /*DASOM_REALSENSE_D435I_H_*/
