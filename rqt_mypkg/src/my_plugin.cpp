/*
  Copyright 2018
*/
#include "my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

double PI = 3.141592;

namespace rqt_mypkg_cpp
{

MyPlugin::MyPlugin()
: rqt_gui_cpp::Plugin()
  , widget_(0)  
{
  setObjectName("C++PluginT");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  ////////////////////////////////////////////////////////////////////////////////////////
  ros::start();
  ros::NodeHandle n;

  gui_init_set = n.createTimer(ros::Duration(0.1), &MyPlugin::gui_init_callback, this);

  AngleSubscriber = n.subscribe("/joint_states", 1, &MyPlugin::AngleSubscriber_Callback, this);
  cam_subscriber = n.subscribe("/dasom/camera_image", 1, &MyPlugin::callbackImage, this);
  EE_subscriber = n.subscribe("/dasom/EE_pose", 1, &MyPlugin::measured_EE_callback, this);
  joystick_sub_ = n.subscribe("/dasom/EE_cmd", 10, &MyPlugin::cmd_EE_callback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = n.subscribe("/phantom/button", 10, &MyPlugin::buttonCallback, this, ros::TransportHints().tcpNoDelay());

  measured_angle.resize(6);
  measured_position.resize(6);
  cmd_position.resize(6);
}

void MyPlugin::shutdownPlugin()
{
  publisher.shutdown();
  AngleSubscriber.shutdown();
  ros::shutdown();

}

void MyPlugin::gui_init_callback(const ros::TimerEvent&) 
{
  ui_.txt_measured_x->setText((QString::number(measured_position[0], 'f', 3)));
  ui_.txt_measured_y->setText((QString::number(measured_position[1], 'f', 3)));
  ui_.txt_measured_z->setText((QString::number(measured_position[2], 'f', 3)));
  ui_.txt_measured_roll->setText((QString::number(measured_position[3], 'f', 3)));
  ui_.txt_measured_pitch->setText((QString::number(measured_position[4], 'f', 3)));
  ui_.txt_measured_yaw->setText((QString::number(measured_position[5], 'f', 3)));


  ui_.txt_cmd_x->setText((QString::number(cmd_position[0], 'f', 3)));
  ui_.txt_cmd_y->setText((QString::number(cmd_position[1], 'f', 3)));
  ui_.txt_cmd_z->setText((QString::number(cmd_position[2], 'f', 3)));
  ui_.txt_cmd_roll->setText((QString::number(cmd_position[3], 'f', 3)));

  ui_.txt_error_x->setText((QString::number(cmd_position[0] - measured_position[0], 'f', 3)));
  ui_.txt_error_y->setText((QString::number(cmd_position[1] - measured_position[1], 'f', 3)));
  ui_.txt_error_z->setText((QString::number(cmd_position[2] - measured_position[2], 'f', 3)));
  ui_.txt_error_roll->setText((QString::number(cmd_position[3] - measured_position[3], 'f', 3)));
  ui_.txt_error_pitch->setText((QString::number(cmd_position[4] - measured_position[4], 'f', 3)));
  ui_.txt_error_yaw->setText((QString::number(cmd_position[5] - measured_position[5], 'f', 3)));
}

void MyPlugin::buttonCallback(const omni_msgs::OmniButtonEvent &msg)
{
  if(msg.grey_button == 1) 
  {
    if(grey_button)
    {
      grey_button = false;
    ui_.lbl_manipulator_mode->setText("Fixed Mode");
    }
    else
    {
      grey_button = true;
    ui_.lbl_manipulator_mode->setText("Gimbaling Mode");
    }
  }
}

void MyPlugin::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

    // OpenCV 를 QImage로 변환
    QImage qimage(cv_ptr->image.data,
                  cv_ptr->image.cols,
                  cv_ptr->image.rows,
                  cv_ptr->image.step,
                  QImage::Format_RGB888);

    // image_frame에 이미지 표시
    ui_.image_frame->setPixmap(QPixmap::fromImage(qimage));
  
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
}

void MyPlugin::AngleSubscriber_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  measured_angle[0] = msg->position.at(0);
  measured_angle[1] = msg->position.at(1);
  measured_angle[2] = msg->position.at(2);
  measured_angle[3] = msg->position.at(3);
  measured_angle[4] = msg->position.at(4);
  measured_angle[5] = msg->position.at(5);
}

void MyPlugin::measured_EE_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  measured_position[0] = msg->linear.x;
  measured_position[1] = msg->linear.y;
  measured_position[2] = msg->linear.z;
  measured_position[3] = msg->angular.x;
  measured_position[4] = msg->angular.y;
  measured_position[5] = msg->angular.z;
}

void MyPlugin::cmd_EE_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_position[0] = msg->linear.x;
  cmd_position[1] = msg->linear.y;
  cmd_position[2] = msg->linear.z;
  cmd_position[3] = M_PI / 2;
  cmd_position[4] = 0;
  cmd_position[5] = 0;
}

}  
// namespace rqt_mypkg_cpp

// #define PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)
PLUGINLIB_EXPORT_CLASS(rqt_mypkg_cpp::MyPlugin, rqt_gui_cpp::Plugin)


