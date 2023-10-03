#ifndef DASOM_CONTROL_GUI_DASOMRQT_H
#define DASOM_CONTROL_GUI_DASOMRQT_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_dasom_rqt.h>
#include <QWidget>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QFile>
#include <QFrame>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <omni_msgs/OmniButtonEvent.h>

#define package_path "/home/choisol/dasom_ws/src/dasom_control_gui"

namespace dasom_control_gui {

class DasomRQT
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  DasomRQT();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();

  cv::Mat conversion_mat_;
  Eigen::VectorXd measured_angle;
  Eigen::VectorXd measured_position;
  Eigen::VectorXd cmd_position;

  bool grey_button;
private:
  Ui::DasomRQTWidget ui_;
  QWidget* widget_;
  ros::Publisher publisher;         //이건 GUI Shutdown 용이라서 건들면 안 됨.
  ros::Publisher pub;
    
  ros::Subscriber AngleSubscriber;
  ros::Subscriber cam_subscriber;
  ros::Subscriber EE_subscriber;
  ros::Subscriber joystick_sub_;
  ros::Subscriber button_sub_;

  ros::Timer gui_init_set;
  QImage qimage_;

  void gui_init_callback(const ros::TimerEvent&);  
  void AngleSubscriber_Callback(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
  void measured_EE_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void cmd_EE_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void buttonCallback(const omni_msgs::OmniButtonEvent &msg);
};
} // namespace
#endif // DASOM_CONTROL_GUI_DASOMRQT_H
