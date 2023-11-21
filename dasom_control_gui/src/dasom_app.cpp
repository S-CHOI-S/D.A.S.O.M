#include "dasom_control_gui/dasom_frame.h"

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "dasom_app", ros::init_options::NoSigintHandler);
  ros::NodeHandlePtr nh_;

  nh_.reset(new ros::NodeHandle);

  ros::Rate loop_rate(250);

  QApplication app(argc, argv);

  QWidget *widget_ = new QWidget;

  dasom_control_gui::DasomFrame ds_frame_(widget_);

  widget_->show();

  return app.exec();
}

