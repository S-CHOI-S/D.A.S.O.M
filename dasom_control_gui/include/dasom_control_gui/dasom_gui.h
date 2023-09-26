#ifndef DASOM_GUI_DASOMRQTT_H
#define DASOM_GUI_DASOMRQTT_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_dasom_rqtt.h>
#include <QWidget>

namespace dasom_gui {

class DasomRQTT
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  DasomRQTT();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::ImageViewWidget ui_;
  QWidget* widget_;
};
} // namespace
#endif // DASOM_GUI_DASOMRQTT_H
