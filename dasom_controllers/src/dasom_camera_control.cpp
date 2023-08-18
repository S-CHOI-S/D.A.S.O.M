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
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &DasomCamControl::buttonCallback, this, ros::TransportHints().tcpNoDelay());
}

void DasomCamControl::joystickCallback(const geometry_msgs::Twist &msg)
{
  haptic_pose[0] = msg.linear.x;
  haptic_pose[1] = msg.linear.y;
  haptic_pose[2] = msg.linear.z;
  haptic_pose[3] = msg.angular.x;
  haptic_pose[4] = msg.angular.y;
  haptic_pose[5] = msg.angular.z;

  ROS_INFO("core_x = %lf, core_y = %lf, core_z = %lf", haptic_pose[0], haptic_pose[2], haptic_pose[1]);
}

void DasomCamControl::buttonCallback(const omni_msgs::OmniButtonEvent &msg)
{
  // grey button 누르면 gimbal_frame 위치 고정(십자가로 나타내기)
  // https://diyver.tistory.com/115 // 글씨 쓰기(gimbal_frame)
  // if) haptic_pose의 위치가 gimbal_frame의 위치와 일치하게 되면 색깔 순간적으로 초록색으로 변화
  // 그 다음에 gimbal_frame 십자가 없어지고 haptic_cmd 십자가만 남도록 하기 

  // ROS_INFO("core_x = %lf, core_y = %lf, core_z = %lf", haptic_pose[0], haptic_pose[2], haptic_pose[1]);
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
  // double t = 0;

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    // ds_cam_ctrl.haptic_pose[0] = 0.1* sin(t/50); 
    // ds_cam_ctrl.haptic_pose[2] = 0.1* sin(t/50); 
    // ds_cam_ctrl.haptic_pose[1] = 0.1* sin(t/50);
    ds_cam.UpdateCamera(1000 * ds_cam_ctrl.haptic_pose[0], 1000 * ds_cam_ctrl.haptic_pose[2], 1000 * ds_cam_ctrl.haptic_pose[1]);

    ros::spinOnce();
    loop_rate.sleep(); 
    // t++;
  }

  return 0;
}