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

  QwtPlotCurve *Xcurve = new QwtPlotCurve("Ext Force X");
  Xcurve->setRenderHint(QwtPlotItem::RenderAntialiased);
  Xcurve->setPen(QPen(Qt::red, 2));

  QTimer dataTimer;
  dataTimer.setInterval(50); // update 1sec = 1000
  int currentTime = 0; // initial time
  QObject::connect(&dataTimer, &QTimer::timeout, [&]()
  {
    QVector<QPointF> dataPoints;

    for (int i = 0; i < 50; ++i) 
    {
      double x = currentTime + i;
      double y = ds_frame_.estimated_force_x;
      dataPoints.append(QPointF(x, y));
    }
    // std::cout<<dataPoints<<std::endl:
    Xcurve->setSamples(dataPoints);
    ds_frame_.ui_.force_plot->replot();

    currentTime += 1;
  });

  dataTimer.start();

  Xcurve->attach(ds_frame_.ui_.force_plot);

  QwtLegend *Xlegend = new QwtLegend;
  Xlegend->setDefaultItemMode(QwtLegendData::ReadOnly);
  ds_frame_.ui_.force_plot->insertLegend(Xlegend, QwtPlot::BottomLegend);
  ds_frame_.ui_.force_plot->setAxisTitle(QwtPlot::xBottom, " ");
  ds_frame_.ui_.force_plot->setAxisScale(QwtPlot::yLeft, -5.0, 5.0);

  widget_->show();

  return app.exec();
}

