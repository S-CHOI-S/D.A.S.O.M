#ifndef DASOM_APP_H
#define DASOM_APP_H

#include <ros/ros.h>
#include <QApplication>
#include "dasom_control_gui/dasom_frame.h"

#define package_path "/home/choisol/dasom_ws/src/dasom_control_gui"

namespace dasom_control_gui {

class DasomApp : public QApplication
{
public:
  ros::NodeHandlePtr nh_;

  DasomApp(int& argc, char** argv);

};
} // namespace
#endif // DASOM_APP_H
