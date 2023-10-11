#include "dasom_control_gui/dasom_frame.h"

namespace dasom_control_gui {

DasomFrame::DasomFrame(QWidget *widget)
: QWidget(widget)
, widget_(widget)
, private_nh_("~")
{
  ui_.setupUi(widget_);
  // widget_->show();

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  initWidget();
  initSubscriber();

  initNodeStatus();
  setBatteryVoltageGauge();
  setEstimatedForcePlot();

  dataTimer = new QTimer(this);
  dataTimer->setInterval(50);
  QObject::connect(dataTimer, SIGNAL(timeout()), this, SLOT(drawEstimatedForcePlot()));
  dataTimer->start();

  // drawEstimatedForcePlot();
}

DasomFrame::~DasomFrame() 
{

}

void DasomFrame::onUpdate()
{
  ros::spinOnce();

  if (!ros::ok())
  {
    close();
  }
}

void DasomFrame::initSubscriber()
{
  palletrone_battery_sub_ = nh_.subscribe("/palletrone/battery", 10, &DasomFrame::batteryVoltageCallback, this);
  estimated_force_sub_ = nh_.subscribe("/dasom/estimated_force", 10, &DasomFrame::estimatedForceCallback, this);
  cam_sub_ = nh_.subscribe("/dasom/camera_image", 10, &DasomFrame::cameraImageCallback, this);

  ds_ctrl_srv_ = nh_.advertiseService("/dasom/node/ds_ctrl", &DasomFrame::dasomControlNodeCallback, this);
  hpt_ctrl_srv_ = nh_.advertiseService("/dasom/node/hpt_ctrl", &DasomFrame::hapticJoystickNodeCallback, this);
  dxl_ctrl_srv_ = nh_.advertiseService("/dasom/node/dxl_ctrl", &DasomFrame::dynamixelControlNodeCallback, this);
}

void DasomFrame::batteryVoltageCallback(const geometry_msgs::Twist &msg)
{
  voltage_value = msg.linear.x;

  QString voltage = QString::number(voltage_value);
  voltageLabel->setText(voltage + " V");
  mSpeedNeedle->setCurrentValue(voltage_value);
}

void DasomFrame::initWidget()
{
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

  ROS_WARN("init widget callback!");
}

void DasomFrame::initNodeStatus()
{
  QString loading_path = QString(package_path) + "/resources/loading_blue.gif";
  movie_ = new QMovie(loading_path);
  ui_.dyn_status->setMovie(movie_);
  ui_.haptic_status->setMovie(movie_);
  ui_.ds_status->setMovie(movie_);
  movie_->start();
}

bool DasomFrame::dasomControlNodeCallback(std_srvs::SetBool::Request  &req,
                                          std_srvs::SetBool::Response &res)
{
  bool status = req.data;

  res.success = true;
  
  if(status) setNodeStatus("ds_status", true);
  else setNodeStatus("ds_status", false);

  return true;
}

bool DasomFrame::hapticJoystickNodeCallback(std_srvs::SetBool::Request  &req,
                                            std_srvs::SetBool::Response &res)
{
  bool status = req.data;

  res.success = true;
  
  if(status) setNodeStatus("haptic_status", true);
  else setNodeStatus("haptic_status", false);
  
  return true;
}

bool DasomFrame::dynamixelControlNodeCallback(std_srvs::SetBool::Request  &req,
                                              std_srvs::SetBool::Response &res)
{
  bool status = req.data;

  res.success = true;
  
  if(status) setNodeStatus("dyn_status", true);
  else setNodeStatus("dyn_status", false);
  
  return true;
}

void DasomFrame::setNodeStatus(QString label_name, bool status)
{
  QString check_path = QString(package_path) + "/resources/check_green.png";
  QString crossmark_path = QString(package_path) + "/resources/crossmark_red.gif";

  if(label_name == "dyn_status")
  {
    if(status)
    {
      QPixmap pixmap;
      pixmap.load(check_path);
      int w = ui_.dyn_status->width();
      int h = ui_.dyn_status->height();
      ui_.dyn_status->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));
    }
    else
    {
      movie_ = new QMovie(crossmark_path);
      movie_->setScaledSize(QSize(ui_.dyn_status->width(), ui_.dyn_status->height()));
      ui_.dyn_status->setMovie(movie_);
      movie_->setSpeed(250);
      movie_->start();
    }
  }
  
  if(label_name == "haptic_status")
  {
    if(status)
    {
      QPixmap pixmap;
      pixmap.load(check_path);
      int w = ui_.haptic_status->width();
      int h = ui_.haptic_status->height();
      ui_.haptic_status->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));
    }
    else
    {
      movie_ = new QMovie(crossmark_path);
      movie_->setScaledSize(QSize(ui_.haptic_status->width(), ui_.haptic_status->height()));
      ui_.haptic_status->setMovie(movie_);
      movie_->setSpeed(250);
      movie_->start();
    }
  }

  if(label_name == "ds_status")
  {
    if(status)
    {
      QPixmap pixmap;
      pixmap.load(check_path);
      int w = ui_.ds_status->width();
      int h = ui_.ds_status->height();
      ui_.ds_status->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));
    }
    else
    {
      movie_ = new QMovie(crossmark_path);
      movie_->setScaledSize(QSize(ui_.ds_status->width(), ui_.ds_status->height()));
      ui_.ds_status->setMovie(movie_);
      movie_->setSpeed(250);
      movie_->start();
    }
  }
}

void DasomFrame::cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

    // OpenCV 를 QImage로 변환
    QImage qimage(cv_ptr->image.data,
                  cv_ptr->image.cols,
                  cv_ptr->image.rows,
                  cv_ptr->image.step,
                  QImage::Format_RGB888);

    // image_frame에 이미지 표시
    ui_.image_frame->setPixmap(QPixmap::fromImage(qimage));
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
}

void DasomFrame::rosNodeStatus()
{
  std::vector<std::string> node_names;
  ros::master::getNodes(node_names);

  std::cout << "Running nodes:" << std::endl;
  for (const std::string& node_name : node_names)
  {
      std::cout << node_name << std::endl;
  }
}

void DasomFrame::setBatteryVoltageGauge()
{
  QcGaugeWidget *mSpeedGauge = new QcGaugeWidget;
  mSpeedGauge->setFixedSize(220, 220);

  QcDegreesItem* speedDegreesItem = mSpeedGauge->addDegrees(40);
  speedDegreesItem->setValueRange(0,24);
  speedDegreesItem->setStep(4);
  mSpeedGauge->addColorBand(30);

  QcValuesItem* speedValuesItem = mSpeedGauge->addValues(65);
  speedValuesItem->setValueRange(0, 24);
  speedValuesItem->setStep(4);

  QString voltage = QString::number(voltage_value);
  voltageLabel = mSpeedGauge->addLabel(28);
  voltageLabel->setText(voltage + " V");

  mSpeedNeedle = mSpeedGauge->addNeedle(25);
  mSpeedNeedle->setColor(Qt::white);
  mSpeedNeedle->setValueRange(0,24);
  
  ui_.battery_gauge->addWidget(mSpeedGauge);

  mSpeedNeedle->setCurrentValue(voltage_value);
}

void DasomFrame::setEstimatedForcePlot()
{
  Xcurve = new QwtPlotCurve("Ext Force X");
  Xcurve->setRenderHint(QwtPlotItem::RenderAntialiased);
  Xcurve->setPen(QPen(Qt::red, 2));

  Xcurve->attach(ui_.force_plot);

  QwtLegend *Xlegend = new QwtLegend;
  Xlegend->setDefaultItemMode(QwtLegendData::ReadOnly);
  ui_.force_plot->insertLegend(Xlegend, QwtPlot::BottomLegend);
  ui_.force_plot->setAxisTitle(QwtPlot::xBottom, " ");
  ui_.force_plot->setAxisScale(QwtPlot::yLeft, -5.0, 5.0);
}

void DasomFrame::drawEstimatedForcePlot()
{
  qDebug() << "Timer timeout event occurred!";
  ROS_WARN("========================");

  QVector<double> xValues;
  for (int i = 0; i < 50; ++i) 
  {
    double x = currentTime + i;
    double y = returnEstimatedForceX();
    xValues.append(x);
    // ROS_INFO("time = %lf, value = %lf",x,y);
    // ROS_WARN("====================================");
    ROS_INFO("Callback = %lf, Return = %lf", x, returnEstimatedForceX());
  }
  // Xcurve->setSamples(dataPoints);
  // ui_.force_plot->replot();

  // if (xValues.size() >= 50) 
  // {    
    Xcurve->setSamples(xValues, yValues);
    ui_.force_plot->replot();
    // xValues.clear(); // x 축 값 초기화
    // yValues.clear(); // y 축 값 초기화
  // }
  ROS_WARN("currentTime = %d, size = %d", currentTime, xValues.size());

  currentTime += 1;
}

void DasomFrame::estimatedForceCallback(const geometry_msgs::WrenchStamped &msg)
{
  estimated_force_x = msg.wrench.force.x;
  estimated_force_y = msg.wrench.force.y;
  estimated_force_z = msg.wrench.force.z;

  yValues.append(estimated_force_x);

  // if(yValues.size() > 50)
  // {
  //   yValues.clear();
  // }

  // ROS_INFO("Callback = %lf",estimated_force_x);

} // namespace dasom_control_gui

double DasomFrame::returnEstimatedForceX()
{
  // ROS_INFO("Return = %lf",estimated_force_x);
  return estimated_force_x;
}

}