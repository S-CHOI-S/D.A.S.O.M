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

#include "dasom_controllers/dasom_camera_control_palletrone.h"

DasomCamPalletrone::DasomCamPalletrone()
: node_handle_(""), it_(node_handle_)
, DasomCam(pub, 0) // camera cam이면 0, 다른 webcam이면 그거에 맞춰서!
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  reInitializePublisher(pub);
}

DasomCamPalletrone::~DasomCamPalletrone()
{
  ROS_INFO("Bye [ DasomCamPalletrone ]!");
  ros::shutdown();
}

void DasomCamPalletrone::initPublisher()
{
  pub = it_.advertise(robot_name_ + "/camera_image/palletrone", 1);
}

void DasomCamPalletrone::initSubscriber()
{
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 10, &DasomCamPalletrone::joystickCallback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &DasomCamPalletrone::buttonCallback, this, ros::TransportHints().tcpNoDelay());
  gimbal_sub_ = node_handle_.subscribe("/dasom/global_EE_frame/world", 10, &DasomCamPalletrone::gimbalCallback, this, ros::TransportHints().tcpNoDelay());
}

void DasomCamPalletrone::gimbalCallback(const geometry_msgs::PoseStamped &msg)
// global_EE_pose 받아오기
{
  global_EE_tf[0] = msg.pose.position.x;
  global_EE_tf[1] = msg.pose.position.y;
  global_EE_tf[2] = msg.pose.position.z;

  global_EE_tf[3] = msg.pose.orientation.x;
  global_EE_tf[4] = msg.pose.orientation.y;
  global_EE_tf[5] = msg.pose.orientation.z;
  global_EE_tf[6] = msg.pose.orientation.w;
}

void DasomCamPalletrone::joystickCallback(const geometry_msgs::Twist &msg)
{
  haptic_position[0] = msg.linear.x;
  haptic_position[1] = msg.linear.y;
  haptic_position[2] = msg.linear.z;
}

void DasomCamPalletrone::buttonCallback(const omni_msgs::OmniButtonEvent &msg)
{
  if(msg.grey_button == 1)
  {
    grey = true;

    if(grey_button == 0 && grey == true)
    {
      grey_button++;
      ROS_INFO("Grey 0: Command mode");

      grey = false;
    }
    else if(grey_button == 1)
    {
      grey_button++;
      ROS_INFO("Grey 1: Gimbaling mode");
    }
    else if(grey_button == 2)
    {
      grey_button = 0;
      ROS_INFO("Grey 2: Gimbaling + Command mode");
    }
  }
}

void DasomCamPalletrone::update()
{
  UpdateCameraPalletrone();
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dasom_camera_control_palletrone");
  DasomCamPalletrone ds_cam_pallet;

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ds_cam_pallet.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
