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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sstream>

#define black cv::Scalar::all(0)
#define white cv::Scalar::all(255)
#define blue cv::Scalar(255,0,0)
#define green cv::Scalar(0,255,0)
#define red cv::Scalar(0,0,255)
#define orange cv::Scalar(34,128,242)
#define pink cv::Scalar(243,109,247)

namespace dasom
{
  class DasomCam
  {
  public:
    DasomCam(image_transport::Publisher& publisher, int cam_num_);
    ~DasomCam();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    // For gimbaling
    bool gimbalcommand_safe = false;

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
    void test();
    void reInitializePublisher(image_transport::Publisher& publisher);
    void UpdateCameraCommand(Eigen::Vector3d core);
    void UpdateCameraGimbal(Eigen::Vector3d core, Eigen::Vector3d gimbal);
    void UpdateCameraGimbalCommandSafety(Eigen::Vector3d core, Eigen::Vector3d gimbal);
    void UpdateCameraGimbalCommand(Eigen::Vector3d core, Eigen::Vector3d gimbal);
    void UpdateCameraPalletrone();

  private:
    /*****************************************************************************
    ** ROS NodeHandle
    *****************************************************************************/
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    /*****************************************************************************
    ** ROS Publishers
    *****************************************************************************/
    image_transport::Publisher cam_pub_;

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    // For camera
    cv::VideoCapture cap;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    // For gimaling
    int i = 0;

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
    void initCamera(int cam_num);
    cv::Mat flipCamera(cv::Mat frame);
    cv::Mat rotateCamera(cv::Mat frame);
    cv::Mat drawCoordinate(cv::Mat frame);
    cv::Mat drawHapticJoystick(cv::Mat frame, Eigen::Vector3d core);
    cv::Mat drawHapticPosition(cv::Mat frame, Eigen::Vector3d core);
    cv::Mat drawGimbalCross(Eigen::Vector3d gimbal, cv::Scalar color);
    cv::Mat drawMovingGimbalCross(cv::Mat frame, Eigen::Vector3d core);
    cv::Mat drawPalletroneCoordinate(cv::Mat frame);
    cv::Mat drawManipulatorWorkspace(cv::Mat frame);
    cv::Mat drawPalletroneWorkspace(cv::Mat frame);
    void DetectLightBulb();
  };
} // namespace DASOM

#endif /*DASOM_CAMERA_H_*/
