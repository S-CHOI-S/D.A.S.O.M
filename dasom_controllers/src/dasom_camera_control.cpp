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
  
  global_EE_tf.resize(7);
}

DasomCamControl::~DasomCamControl()
{
  ROS_INFO("Bye [ DasomCamControl ]!");
  ros::shutdown();
}

void DasomCamControl::initPublisher()
{
  pub = it_.advertise(robot_name_ + "/camera_image/dasom", 1);
}

void DasomCamControl::initSubscriber()
{
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 10, &DasomCamControl::joystickCallback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &DasomCamControl::buttonCallback, this, ros::TransportHints().tcpNoDelay());
  gimbal_sub_ = node_handle_.subscribe("/dasom/tf/global_fixed_gimbal_EE_pose", 10, &DasomCamControl::gimbalCallback, this, ros::TransportHints().tcpNoDelay());
}

void DasomCamControl::gimbalCallback(const geometry_msgs::PoseStamped &msg)
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

void DasomCamControl::joystickCallback(const geometry_msgs::Twist &msg)
{
  haptic_position[0] = msg.linear.x;
  haptic_position[1] = msg.linear.y;
  haptic_position[2] = msg.linear.z;
}

void DasomCamControl::buttonCallback(const omni_msgs::OmniButtonEvent &msg)
{
  if(msg.grey_button == 1)
  {
    grey = true;

    if(grey_button == 0 && grey == true)
    {
      grey_button++;
      ROS_INFO("Grey 1: Gimbaling mode");

      gimbal_position_tf = haptic_position;

      grey = false;
    }
    else if(grey_button == 1)
    {
      grey_button++;
      ROS_INFO("Grey 2: Gimbaling + Command mode");
    }
    else if(grey_button == 2)
    {
      grey_button = 0;
      ROS_INFO("Grey 0: Command mode");
    }
  }
}

void DasomCamControl::update()
{
  if(grey_button == 0)
  {
    // ROS_INFO("Grey 0: Command mode");
    UpdateCameraCommand(1000 * haptic_position);
  }

  else if(grey_button == 1) 
  {
    // ROS_INFO("Grey 1: Gimbaling mode");
    UpdateCameraGimbal(1000 * haptic_position, 1000 * gimbal_position_tf); 
  }

  else if(grey_button == 2) 
  {
    // ROS_INFO("Grey 2: Gimbaling + Command mode");
    if(gimbalcommand_safe == false)
    {
      UpdateCameraGimbalCommandSafety(1000 * haptic_position, 1000 * gimbal_position_tf);
    }
    else 
    {
      UpdateCameraGimbalCommand(1000 * haptic_position, 1000 * gimbal_position_tf);
    }
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dasom_camera_control");
  DasomCamControl ds_cam_ctrl;

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ds_cam_ctrl.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
