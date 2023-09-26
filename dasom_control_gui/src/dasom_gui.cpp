#include "dasom_control_gui/dasom_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace dasom_gui {

DasomRQTT::DasomRQTT()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("DasomRQTT");
}

void DasomRQTT::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  ui_.image_frame->installEventFilter(this);
}

void DasomRQTT::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void DasomRQTT::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void DasomRQTT::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace

PLUGINLIB_EXPORT_CLASS(dasom_gui::DasomRQTT, rqt_gui_cpp::Plugin)
