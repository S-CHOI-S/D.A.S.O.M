#include "dasom_control_gui/dasom_frame.h"

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "dasom_app", ros::init_options::NoSigintHandler);
  ros::NodeHandlePtr nh_;

  nh_.reset(new ros::NodeHandle);

  ros::Rate loop_rate(250);

  QApplication app(argc, argv);

  QWidget *widget_ = new QWidget;

  Ui::DasomRQTWidget ui_;
  // dasom_control_gui::DasomFrame ds_frame_(widget_);
  ui_.setupUi(widget_);

  QString background_path = "background-image: url(" + QString(package_path) + "/resources/424040.png);";
  widget_->setStyleSheet(background_path);

  QString MRL_logo_path = QString(package_path) + "/resources/MRL_logo_make_white.png";
  QPixmap pixmap;
  pixmap.load(MRL_logo_path);
  int w = ui_.MRL_logo->width();
  int h = ui_.MRL_logo->height();
  ui_.MRL_logo->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));

  QString loading_path = QString(package_path) + "/resources/loading_blue.gif";

  QPixmap pixmap_load;
  pixmap_load.load(loading_path);
  w = ui_.dyn_status->width();
  h = ui_.dyn_status->height();
  ui_.dyn_status->setPixmap(pixmap_load.scaled(w,h,Qt::KeepAspectRatio));
  ui_.haptic_status->setPixmap(pixmap_load.scaled(w,h,Qt::KeepAspectRatio));
  ui_.camera_status->setPixmap(pixmap_load.scaled(w,h,Qt::KeepAspectRatio));

  QMovie movie(loading_path);
  ui_.dyn_status->setMovie(&movie);
  ui_.haptic_status->setMovie(&movie);
  ui_.camera_status->setMovie(&movie);
  movie.start();

  QString check_path = QString(package_path) + "/resources/check_green.png";
  QPixmap pixmap_check;
  pixmap_check.load(check_path);
  w = ui_.check_status->width();
  h = ui_.check_status->height();
  ui_.check_status->setPixmap(pixmap_check.scaled(w,h,Qt::KeepAspectRatio));

  QString crossmark_path = QString(package_path) + "/resources/crossmark_red.gif";
  QPixmap pixmap_crossmark;
  pixmap_crossmark.load(crossmark_path);
  w = ui_.crossmark_status->width();
  h = ui_.crossmark_status->height();
  ui_.crossmark_status->setPixmap(pixmap_crossmark.scaled(w,h,Qt::KeepAspectRatio));
  QMovie movie_crossmark(crossmark_path);
  ui_.crossmark_status->setMovie(&movie_crossmark);
  movie_crossmark.setSpeed(250);
  movie_crossmark.start();
//////////////////////////////////////////////////////////////////////////////
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
      double y = qrand() % 100;
      dataPoints.append(QPointF(x, y));
    }

    Xcurve->setSamples(dataPoints);
    ui_.force_plot->replot();

    currentTime += 1;
  });

  dataTimer.start();

  Xcurve->attach(ui_.force_plot);

  QwtLegend *Xlegend = new QwtLegend;
  Xlegend->setDefaultItemMode(QwtLegendData::ReadOnly);
  ui_.force_plot->insertLegend(Xlegend, QwtPlot::BottomLegend);
  ui_.force_plot->setAxisTitle(QwtPlot::xBottom, " ");

  // Custom Gauge
  QcGaugeWidget *mSpeedGauge = new QcGaugeWidget;
  mSpeedGauge->setFixedSize(220, 220);

  QcDegreesItem* speedDegreesItem = mSpeedGauge->addDegrees(40);
  speedDegreesItem->setValueRange(0,24);
  speedDegreesItem->setStep(4);
  mSpeedGauge->addColorBand(30);

  QcValuesItem* speedValuesItem = mSpeedGauge->addValues(65);
  speedValuesItem->setValueRange(0, 24);
  speedValuesItem->setStep(4);

  double voltage_value = 20;
  QString voltage = QString::number(voltage_value);
  mSpeedGauge->addLabel(28)->setText(voltage + " V");

  QcNeedleItem *mSpeedNeedle = mSpeedGauge->addNeedle(25);
  mSpeedNeedle->setColor(Qt::white);
  mSpeedNeedle->setValueRange(0,24);
  
  ui_.battery_gauge->addWidget(mSpeedGauge);

  mSpeedNeedle->setCurrentValue(voltage_value);

  widget_->show();

  return app.exec();
}

