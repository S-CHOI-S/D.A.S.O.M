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

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/SetBool.h>

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

  void rosNodeStatus();

private slots:
  void onUpdate();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  QWidget *widget_;
  // Ui::DasomRQTWidget ui_;

  QTimer* update_timer_;
  QMovie *movie_;

  QcGaugeWidget *mSpeedGauge;
  QcNeedleItem *mSpeedNeedle;
  QcLabelItem *voltageLabel;

  QwtPlotCurve *Xcurve;

  ros::Subscriber palletrone_battery_sub_;
  ros::Subscriber estimated_force_sub_;
  ros::Subscriber cam_sub_;

  ros::ServiceServer ds_ctrl_srv_;
  ros::ServiceServer hpt_ctrl_srv_;
  ros::ServiceServer dxl_ctrl_srv_;

  int ds_node_status = 0;
  int currentTime = 0;
  double voltage_value = 15;

  void initSubscriber();
  void batteryVoltageCallback(const geometry_msgs::Twist &msg);
  void initWidget();
  bool dasomControlNodeCallback(std_srvs::SetBool::Request  &req,
                                std_srvs::SetBool::Response &res);
  bool hapticJoystickNodeCallback(std_srvs::SetBool::Request  &req,
                                  std_srvs::SetBool::Response &res);
  bool dynamixelControlNodeCallback(std_srvs::SetBool::Request  &req,
                                    std_srvs::SetBool::Response &res);
  void setNodeStatus(QString label_name, bool status);
  void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void initNodeStatus();
  void setBatteryVoltageGauge();
  void setEstimatedForcePlot();
  void drawEstimatedForcePlot();
  void estimatedForceCallback(const geometry_msgs::WrenchStamped &msg);
};
} // namespace
#endif // DASOM_FRAME_H
