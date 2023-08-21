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
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  ds_cam_ = new DasomCam(pub, 0);

  haptic_pose.resize(6);
  gimbal_tf.resize(7);
  global_EE_tf.resize(7);
}

DasomCamControl::~DasomCamControl()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void DasomCamControl::initPublisher()
{
  pub = it_.advertise("/dasom/camera_image", 1);
}

void DasomCamControl::initSubscriber()
{
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 10, &DasomCamControl::joystickCallback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &DasomCamControl::buttonCallback, this, ros::TransportHints().tcpNoDelay());
  gimbal_sub_ = node_handle_.subscribe("/dasom/global_EE_frame/world", 10, &DasomCamControl::gimbalCallback, this, ros::TransportHints().tcpNoDelay());
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
  if(msg.grey_button == 1)
  {
    grey = true;

    if(grey_button == 0 && grey == true)
    {
      grey_button++;
      ROS_INFO("Grey: Gimbaling mode");

      gimbal_tf = global_EE_tf;

      ds_cam_->DrawGimbalCross(gimbal_tf[0],gimbal_tf[1],gimbal_tf[2]);

      grey = false;

      // ROS_INFO("GIMBAL_TF = %lf, %lf, %lf, %lf, %lf, %lf", gimbal_tf[0], gimbal_tf[1], gimbal_tf[2], gimbal_tf[3], gimbal_tf[4], gimbal_tf[5]);
    }
    else if(grey_button == 1 && grey == true)
    {
      grey_button++;
      ROS_INFO("Grey: Gimbaling + Command mode");
    }
    else if(grey_button == 2 && grey == true)
    {
      grey_button = 0;
      ROS_INFO("Grey: Command mode");
    }
  }

  // ROS_INFO("core_x = %lf, core_y = %lf, core_z = %lf", haptic_pose[0], haptic_pose[2], haptic_pose[1]);
}

void DasomCamControl::L()
{
  // ds_cam_->test();
  ds_cam_->UpdateCamera(1000 * haptic_pose[0], 
                                      1000 * haptic_pose[2], 
                                      1000 * haptic_pose[1]);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dasom_camera_control");
  DasomCamControl ds_cam_ctrl;

  // ds_cam_ctrl.ds_cam_;

  double t = 0;

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ds_cam_ctrl.haptic_pose[0] = 0.1* sin(t/50); 
    ds_cam_ctrl.haptic_pose[2] = 0.1* sin(t/50); 
    ds_cam_ctrl.haptic_pose[1] = 0.1* sin(t/50);

    // ds_cam_ctrl.ds_cam_->UpdateCamera(1000 * ds_cam_ctrl.haptic_pose[0], 
    //                                   1000 * ds_cam_ctrl.haptic_pose[2], 
    //                                   1000 * ds_cam_ctrl.haptic_pose[1]);

    ds_cam_ctrl.L();
    ros::spinOnce();
    loop_rate.sleep(); 
    t++;
  }

  return 0;
}