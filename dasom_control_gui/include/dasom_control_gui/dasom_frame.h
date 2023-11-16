#ifndef DASOM_FRAME_H
#define DASOM_FRAME_H

#include <ros/ros.h>
#include <ros/master.h>
#include <iostream>
#include <vector>
#include <QtGui>
// #include <QApplication>
#include <QPalette>
#include <QLabel>
#include <QTimer>
#include <QObject>
#include <QMovie>
#include <QImage>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "ui_dasom_rqt.h"
#include "qcgaugewidget.h"
#include <pluginlib/class_list_macros.h>
#include <rqt_gui_cpp/plugin.h>
#include <qwt_plot.h>
#include <qwt_plot_canvas.h>
#include <qwt_scale_draw.h>
#include <qwt_text.h>
#include <qwt_legend.h>
#include <qpen.h>
#include <qwt_scale_div.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_curve.h>
#include <qwt_symbol.h>
#include <qwt_dial.h>
#include "qwt_dial_needle.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int16.h>
#include <std_srvs/SetBool.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define package_path "/home/choisol/dasom_ws/src/dasom_control_gui"

namespace dasom_control_gui {

class DasomFrame : public QWidget
{
  Q_OBJECT
public:
  DasomFrame(QWidget *widget = 0);
  ~DasomFrame();

  Ui::DasomRQTWidget ui_;

  double estimated_force_x = 0;
  double estimated_force_y = 0;
  double estimated_force_z = 0;

private slots:
  void onUpdate();
  void rosNodeStatus();
  void drawEstimatedForcePlot();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Timer setTXT;

  QWidget *widget_;
  // Ui::DasomRQTWidget ui_;

  QTimer* update_timer_;
  QTimer* dataTimer;
  QTimer* node_timer_;

  QMovie *movie_;

  QcGaugeWidget *mSpeedGauge;
  QcNeedleItem *mSpeedNeedle;
  QcLabelItem *voltageLabel;

  QwtPlotCurve *Xcurve;

  ros::Subscriber palletrone_battery_sub_;
  ros::Subscriber estimated_force_sub_;
  ros::Subscriber cam_ds_sub_;
  ros::Subscriber cam_pt_sub_;
  ros::Subscriber EE_measured_sub_;
  ros::Subscriber EE_command_sub_;
  ros::Subscriber battery_checker;

  bool ds = false;
  bool hpt = false;
  bool dyn = false;
  bool ds_found = false;
  bool hpt_found = false;
  bool dyn_found = false;

  QVector<double> yValues;

  int ds_node_status = 0;
  int currentTime = 0;
  double voltage_value = 15;

  Eigen::VectorXd ds_measured_position;
  Eigen::VectorXd ds_cmd_position;

  void initSubscriber();
  void dsEEMeasuredCallback(const geometry_msgs::Twist &msg);
  void dsEECommandCallback(const geometry_msgs::Twist &msg);
  void batteryVoltageCallback(const geometry_msgs::Twist &msg);
  void initWidget();
  void setNodeStatus(QString label_name, bool status);
  void cameraImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
  void cameraPtImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
  void batteryCallback(const std_msgs::Int16& msg);
  void initNodeStatus();
  void setBatteryVoltageGauge();
  void setEstimatedForcePlot();
  // void drawEstimatedForcePlot();
  void estimatedForceCallback(const geometry_msgs::WrenchStamped &msg);
  double returnEstimatedForceX();
  void setManipulatorTXT(const ros::TimerEvent&);
  void clickedKillButton();
};
} // namespace
#endif // DASOM_FRAME_H
