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

  node_timer_ = new QTimer(this);
  node_timer_->setInterval(1000);
  node_timer_->start();
  connect(node_timer_, SIGNAL(timeout()), this, SLOT(rosNodeStatus()));

  initWidget();
  initSubscriber();

  initNodeStatus();
  setBatteryVoltageGauge();
  setEstimatedForcePlot();

  ds_measured_position.resize(6);
  ds_cmd_position.resize(6);
  ds_measured_position << 0, 0, 0, 0, 0, 0;
  ds_cmd_position << 0, 0, 0, 0, 0, 0;

  pt_measured_position.resize(6);
  pt_cmd_position.resize(6);
  pt_measured_position << 0, 0, 0, 0, 0, 0;
  pt_cmd_position << 0, 0, 0, 0, 0, 0;

  data_log_data.data.resize(124);
  data_log_data.data = {0,};

  dataTimer = new QTimer(this);
  dataTimer->setInterval(50);
  QObject::connect(dataTimer, SIGNAL(timeout()), this, SLOT(drawEstimatedForcePlot()));
  dataTimer->start();

  setTXT = nh_.createTimer(ros::Duration(0.1), &DasomFrame::setManipulatorTXT, this);

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
  cam_ds_sub_ = nh_.subscribe("/dasom/camera_image/dasom/compressed", 10, &DasomFrame::cameraImageCallback, this);
  cam_pt_sub_ = nh_.subscribe("/dasom/camera_image/palletrone/compressed", 10, &DasomFrame::cameraPtImageCallback, this);
  EE_measured_sub_ = nh_.subscribe("/dasom/EE_pose", 10, &DasomFrame::dsEEMeasuredCallback, this);
  EE_command_sub_ = nh_.subscribe("/dasom/test_Pub", 10, &DasomFrame::dsEECommandCallback, this);
  battery_checker = nh_.subscribe("/battery_voltage", 10, &DasomFrame::batteryCallback, this);
  data_log_sub_ = nh_.subscribe("/data_log", 10, &DasomFrame::globalEEPoseCallback, this);
  pt_cmd_sub_ = nh_.subscribe("/data_log", 10, &DasomFrame::ptEECommandCallback, this);
  pt_meas_sub_ = nh_.subscribe("/data_log", 10, &DasomFrame::ptEEMeasuredCallback, this);
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

void DasomFrame::cameraImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // OpenCV 를 QImage로 변환
    QImage qimage(cv_ptr->image.data,
                  cv_ptr->image.cols,
                  cv_ptr->image.rows,
                  cv_ptr->image.step,
                  QImage::Format_RGB888);

    // 이미지가 BGR로 변환된 경우, RGB로 변환
    if (qimage.format() == QImage::Format_RGB888)
    {
      qimage = qimage.rgbSwapped(); // BGR에서 RGB로 변환
    }

    QRect croppedRect(0, 10, 480, 600);
    qimage = qimage.copy(croppedRect);

    qimage = qimage.scaledToWidth(ui_.image_frame->width());
    
    ui_.image_frame->setPixmap(QPixmap::fromImage(qimage));
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
}

void DasomFrame::cameraPtImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // OpenCV 를 QImage로 변환
    QImage qimage(cv_ptr->image.data,
                  cv_ptr->image.cols,
                  cv_ptr->image.rows,
                  cv_ptr->image.step,
                  QImage::Format_RGB888);

    // 이미지가 BGR로 변환된 경우, RGB로 변환
    if (qimage.format() == QImage::Format_RGB888)
    {
      qimage = qimage.rgbSwapped(); // BGR에서 RGB로 변환
    } 

    QSize newSize(ui_.image_frame_2->width(), ui_.image_frame_2->height());
    qimage = qimage.scaled(newSize, Qt::KeepAspectRatio);

    qimage = qimage.scaledToWidth(ui_.image_frame_2->width());

    ui_.image_frame_2->setPixmap(QPixmap::fromImage(qimage));
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }
}

void DasomFrame::batteryCallback(const std_msgs::Float32& msg)
{
  voltage_value = msg.data;

	mSpeedNeedle->setCurrentValue(voltage_value);
}

void DasomFrame::rosNodeStatus()
{
  std::vector<std::string> node_names;
  ros::master::getNodes(node_names);

  std::string dyn_node_string = "/torque_ctrl_6DOF";
  std::string ds_node_string = "/dasom_manipulator_control";
  std::string haptic_node_string = "/omni_state";

  // std::cout << "Running nodes:" << std::endl;
  for (const std::string& node_name : node_names)
  {
    // std::cout << node_name << std::endl;

    if(node_name == dyn_node_string)
    {
      setNodeStatus("dyn_status", true);
      dyn = true;
      dyn_found = true;
    }

    if(node_name == ds_node_string) 
    {
      setNodeStatus("ds_status", true);
      ds = true;
      ds_found = true;
    }
    
    if(node_name == haptic_node_string)
    {
      setNodeStatus("haptic_status", true);
      hpt = true;
      hpt_found = true;
    }
  }

  if((dyn_found == false) && (dyn == true))
  {
    setNodeStatus("dyn_status", false);
  }

  if((ds_found == false) && (ds == true))
  {
    setNodeStatus("ds_status", false);
  }

  if((hpt_found == false) && (hpt == true))
  {
    setNodeStatus("haptic_status", false);
  }

  ds_found = false;
  hpt_found = false;
  dyn_found = false;
}

void DasomFrame::globalEEPoseCallback(const std_msgs::Float64MultiArray &msg)
{
  data_log_data.data[0] = msg.data[12];
  data_log_data.data[1] = msg.data[13];
  data_log_data.data[2] = msg.data[14];
  data_log_data.data[3] = msg.data[15];
  data_log_data.data[4] = msg.data[16];
  data_log_data.data[5] = msg.data[17];

  data_log_data.data[6] = msg.data[18];
  data_log_data.data[7] = msg.data[19];
  data_log_data.data[8] = msg.data[20];
  data_log_data.data[9] = msg.data[21];
  data_log_data.data[10] = msg.data[22];
  data_log_data.data[11] = msg.data[23];
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
  // Xcurve = new QwtPlotCurve("Ext Force X");
  // Xcurve->setRenderHint(QwtPlotItem::RenderAntialiased);
  // Xcurve->setPen(QPen(Qt::red, 2));

  // Xcurve->attach(ui_.force_plot);

  // QwtLegend *Xlegend = new QwtLegend;
  // Xlegend->setDefaultItemMode(QwtLegendData::ReadOnly);
  // ui_.force_plot->insertLegend(Xlegend, QwtPlot::BottomLegend);
  // ui_.force_plot->setAxisTitle(QwtPlot::xBottom, " ");
  // ui_.force_plot->setAxisScale(QwtPlot::yLeft, -5.0, 5.0);
}

void DasomFrame::drawEstimatedForcePlot()
{
  // QVector<double> xValues;
  // for (int i = 0; i < 50; ++i) 
  // {
  //   double x = currentTime + i;
  //   double y = returnEstimatedForceX();
  //   xValues.append(x);
  //   // ROS_INFO("time = %lf, value = %lf",x,y);
  //   // ROS_WARN("====================================");
  //   // ROS_INFO("Callback = %lf, Return = %lf", x, returnEstimatedForceX());
  // }
  // // Xcurve->setSamples(dataPoints);
  // // ui_.force_plot->replot();

  // // if (xValues.size() >= 50) 
  // // {    
  //   Xcurve->setSamples(xValues, yValues);
  //   ui_.force_plot->replot();
  //   // xValues.clear(); // x 축 값 초기화
  //   // yValues.clear(); // y 축 값 초기화
  // // }
  // // ROS_WARN("currentTime = %d, size = %d", currentTime, xValues.size());

  // currentTime += 1;
}

void DasomFrame::estimatedForceCallback(const geometry_msgs::WrenchStamped &msg)
{
  estimated_force_x = abs(msg.wrench.force.x);
  estimated_force_y = abs(msg.wrench.force.y);
  estimated_force_z = abs(msg.wrench.force.z);

  ui_.progressBar_X->setStyleSheet("border: 2px solid #FF0000; border-radius: 5px; background-color: lightgrey;"
                              "QProgressBar::chunk { background-color: #37CDFA; width: 10px; }");

  ui_.progressBar_X->setValue(estimated_force_x);
}

double DasomFrame::returnEstimatedForceX()
{
  // ROS_INFO("Return = %lf",estimated_force_x);
  return estimated_force_x;
}

void DasomFrame::dsEEMeasuredCallback(const geometry_msgs::Twist &msg)
{
  ds_measured_position[0] = msg.linear.x;
  ds_measured_position[1] = msg.linear.y;
  ds_measured_position[2] = msg.linear.z;
  ds_measured_position[3] = msg.angular.x;
  ds_measured_position[4] = msg.angular.y;
  ds_measured_position[5] = msg.angular.z;
}

void DasomFrame::dsEECommandCallback(const geometry_msgs::Twist &msg)
{
  ds_cmd_position[0] = msg.linear.x;
  ds_cmd_position[1] = msg.linear.y;
  ds_cmd_position[2] = msg.linear.z;
  ds_cmd_position[3] = msg.angular.x;
  ds_cmd_position[4] = msg.angular.y;
  ds_cmd_position[5] = msg.angular.z;
}

void DasomFrame::ptEEMeasuredCallback(const std_msgs::Float64MultiArray &msg)
{
  pt_measured_position[0] = msg.data[64];
  pt_measured_position[1] = msg.data[65];
  pt_measured_position[2] = msg.data[66];
  pt_measured_position[3] = msg.data[58];
  pt_measured_position[4] = msg.data[59];
  pt_measured_position[5] = msg.data[60];
}

void DasomFrame::ptEECommandCallback(const std_msgs::Float64MultiArray &msg)
{
  pt_cmd_position[0] = msg.data[61];
  pt_cmd_position[1] = msg.data[62];
  pt_cmd_position[2] = msg.data[63];
  pt_cmd_position[3] = msg.data[55];
  pt_cmd_position[4] = msg.data[56];
  pt_cmd_position[5] = msg.data[57];
}

void DasomFrame::setManipulatorTXT(const ros::TimerEvent&) 
{
  // For DASOM Manipulator
  ui_.txt_measured_x->setText((QString::number(ds_measured_position[0], 'f', 3)));
  ui_.txt_measured_y->setText((QString::number(ds_measured_position[1], 'f', 3)));
  ui_.txt_measured_z->setText((QString::number(ds_measured_position[2], 'f', 3)));
  ui_.txt_measured_roll->setText((QString::number(ds_measured_position[3], 'f', 3)));
  ui_.txt_measured_pitch->setText((QString::number(ds_measured_position[4], 'f', 3)));
  ui_.txt_measured_yaw->setText((QString::number(ds_measured_position[5], 'f', 3)));

  ui_.txt_cmd_x->setText((QString::number(ds_cmd_position[0], 'f', 3)));
  ui_.txt_cmd_y->setText((QString::number(ds_cmd_position[1], 'f', 3)));
  ui_.txt_cmd_z->setText((QString::number(ds_cmd_position[2], 'f', 3)));
  ui_.txt_cmd_roll->setText((QString::number(ds_cmd_position[3], 'f', 3)));
  ui_.txt_cmd_pitch->setText((QString::number(ds_cmd_position[4], 'f', 3)));
  ui_.txt_cmd_yaw->setText((QString::number(ds_cmd_position[5], 'f', 3)));

  ui_.txt_error_x->setText((QString::number(ds_cmd_position[0] - ds_measured_position[0], 'f', 3)));
  ui_.txt_error_y->setText((QString::number(ds_cmd_position[1] - ds_measured_position[1], 'f', 3)));
  ui_.txt_error_z->setText((QString::number(ds_cmd_position[2] - ds_measured_position[2], 'f', 3)));
  ui_.txt_error_roll->setText((QString::number(ds_cmd_position[3] - ds_measured_position[3], 'f', 3)));
  ui_.txt_error_pitch->setText((QString::number(ds_cmd_position[4] - ds_measured_position[4], 'f', 3)));
  ui_.txt_error_yaw->setText((QString::number(ds_cmd_position[5] - ds_measured_position[5], 'f', 3)));

  // For Palletrone
  ui_.txt_measured_x_2->setText((QString::number(pt_measured_position[0], 'f', 3)));
  ui_.txt_measured_y_2->setText((QString::number(pt_measured_position[1], 'f', 3)));
  ui_.txt_measured_z_2->setText((QString::number(pt_measured_position[2], 'f', 3)));
  ui_.txt_measured_roll_2->setText((QString::number(pt_measured_position[3], 'f', 3)));
  ui_.txt_measured_pitch_2->setText((QString::number(pt_measured_position[4], 'f', 3)));
  ui_.txt_measured_yaw_2->setText((QString::number(pt_measured_position[5], 'f', 3)));

  ui_.txt_cmd_x_2->setText((QString::number(pt_cmd_position[0], 'f', 3)));
  ui_.txt_cmd_y_2->setText((QString::number(pt_cmd_position[1], 'f', 3)));
  ui_.txt_cmd_z_2->setText((QString::number(pt_cmd_position[2], 'f', 3)));
  ui_.txt_cmd_roll_2->setText((QString::number(pt_cmd_position[3], 'f', 3)));
  ui_.txt_cmd_pitch_2->setText((QString::number(pt_cmd_position[4], 'f', 3)));
  ui_.txt_cmd_yaw_2->setText((QString::number(pt_cmd_position[5], 'f', 3)));

  ui_.txt_error_x_2->setText((QString::number(pt_cmd_position[0] - pt_measured_position[0], 'f', 3)));
  ui_.txt_error_y_2->setText((QString::number(pt_cmd_position[1] - pt_measured_position[1], 'f', 3)));
  ui_.txt_error_z_2->setText((QString::number(pt_cmd_position[2] - pt_measured_position[2], 'f', 3)));
  ui_.txt_error_roll_2->setText((QString::number(pt_cmd_position[3] - pt_measured_position[3], 'f', 3)));
  ui_.txt_error_pitch_2->setText((QString::number(pt_cmd_position[4] - pt_measured_position[4], 'f', 3)));
  ui_.txt_error_yaw_2->setText((QString::number(pt_cmd_position[5] - pt_measured_position[5], 'f', 3)));

  // For Global Pose
  ui_.txt_measured_x_3->setText((QString::number(data_log_data.data[0], 'f', 3)));
  ui_.txt_measured_y_3->setText((QString::number(data_log_data.data[1], 'f', 3)));
  ui_.txt_measured_z_3->setText((QString::number(data_log_data.data[2], 'f', 3)));
  ui_.txt_measured_roll_3->setText((QString::number(data_log_data.data[3], 'f', 3)));
  ui_.txt_measured_pitch_3->setText((QString::number(data_log_data.data[4], 'f', 3)));
  ui_.txt_measured_yaw_3->setText((QString::number(data_log_data.data[5], 'f', 3)));

  ui_.txt_cmd_x_3->setText((QString::number(data_log_data.data[6], 'f', 3)));
  ui_.txt_cmd_y_3->setText((QString::number(data_log_data.data[7], 'f', 3)));
  ui_.txt_cmd_z_3->setText((QString::number(data_log_data.data[8], 'f', 3)));
  ui_.txt_cmd_roll_3->setText((QString::number(data_log_data.data[9], 'f', 3)));
  ui_.txt_cmd_pitch_3->setText((QString::number(data_log_data.data[10], 'f', 3)));
  ui_.txt_cmd_yaw_3->setText((QString::number(data_log_data.data[11], 'f', 3)));

  ui_.txt_error_x_3->setText((QString::number(data_log_data.data[0] - data_log_data.data[6], 'f', 3)));
  ui_.txt_error_y_3->setText((QString::number(data_log_data.data[1] - data_log_data.data[7], 'f', 3)));
  ui_.txt_error_z_3->setText((QString::number(data_log_data.data[2] - data_log_data.data[8], 'f', 3)));
  ui_.txt_error_roll_3->setText((QString::number(data_log_data.data[3] - data_log_data.data[9], 'f', 3)));
  ui_.txt_error_pitch_3->setText((QString::number(data_log_data.data[4] - data_log_data.data[10], 'f', 3)));
  ui_.txt_error_yaw_3->setText((QString::number(data_log_data.data[5] - data_log_data.data[11], 'f', 3)));
}

void DasomFrame::clickedKillButton()
{
  // ui_.killButton
}

} // namespace dasom_control_gui