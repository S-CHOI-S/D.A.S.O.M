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

#include "dasom_controllers/dasom_camera_control.h"

DasomCamControl::DasomCamControl()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  haptic_pose.resize(6);
}

DasomCamControl::~DasomCamControl()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void DasomCamControl::initPublisher()
{

}

void DasomCamControl::initSubscriber()
{
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 10, &DasomCamControl::joystickCallback, this, ros::TransportHints().tcpNoDelay());
}

void DasomCamControl::joystickCallback(const geometry_msgs::Twist &msg)
{
  haptic_pose[0] = msg.linear.x;
  haptic_pose[1] = msg.linear.y;
  haptic_pose[2] = msg.linear.z;
  haptic_pose[3] = msg.angular.x;
  haptic_pose[4] = msg.angular.y;
  haptic_pose[5] = msg.angular.z;

  ROS_INFO("core_x = %lf, core_y = %lf", haptic_pose[0], haptic_pose[1]);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dasom_camera_control");
  DasomCamControl ds_cam_ctrl;

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/dasom/camera_image", 1);

  DasomCam ds_cam(pub, 0);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ds_cam.UpdateCamera(1000 * ds_cam_ctrl.haptic_pose[0], 1000 * ds_cam_ctrl.haptic_pose[1]);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}