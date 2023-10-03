#ifndef DASOM_FRAME_H
#define DASOM_FRAME_H

#include <ros/ros.h>
#include <QtGui>
// #include <QApplication>
#include <QPalette>
#include <QLabel>
#include <QTimer>
#include <QObject>
#include <QMovie>
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

#define package_path "/home/choisol/dasom_ws/src/dasom_control_gui"

namespace dasom_control_gui {

class DasomFrame : public QWidget
{
  Q_OBJECT
public:
  DasomFrame(QWidget *widget = 0);
  ~DasomFrame();

  double voltage_value = 15;

private slots:
  void onUpdate();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  QWidget *widget_;
  Ui::DasomRQTWidget ui_;

  QTimer* update_timer_;

  ros::Subscriber palletrone_battery_sub_;
  ros::Subscriber dasom_node_sub_;

  int ds_node_status = 0;

  void initSubscriber();
  void batteryVoltageCallback(const geometry_msgs::Twist &msg);
  void initWidget();
  void dasomControlNodeCallback(const geometry_msgs::Twist &msg);
  void setNodeStatus(int node_status);

};
} // namespace
#endif // DASOM_APP_H
