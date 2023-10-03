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
    dasom_node_sub_ = nh_.subscribe("/palletrone/battery", 10, &DasomFrame::dasomControlNodeCallback, this);
}

void DasomFrame::batteryVoltageCallback(const geometry_msgs::Twist &msg)
{
    voltage_value = msg.linear.x;
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
    QMovie movie(loading_path);
    ui_.dyn_status->setMovie(&movie);
    ui_.haptic_status->setMovie(&movie);
    ui_.camera_status->setMovie(&movie);
    movie.start();
}

void DasomFrame::dasomControlNodeCallback(const geometry_msgs::Twist &msg)
{
    ds_node_status = msg.linear.x;

    setNodeStatus(ds_node_status);
}

void DasomFrame::setNodeStatus(int node_status)
{
  QString check_path = QString(package_path) + "/resources/check_green.png";
  QString crossmark_path = QString(package_path) + "/resources/crossmark_red.gif";

  
//   ui_.haptic_status->setPixmap(pixmap_load.scaled(w,h,Qt::KeepAspectRatio));
//   ui_.camera_status->setPixmap(pixmap_load.scaled(w,h,Qt::KeepAspectRatio));
// ROS_ERROR("%d", ds_node_status);
//     if(node_status < 5)
//     {
//         QMovie movie(loading_path);
//         ROS_INFO("AAAAAA");
//         ui_.dyn_status->setMovie(&movie);
//         ROS_INFO("BBBBBB");
//         ui_.haptic_status->setMovie(&movie);
//         ROS_INFO("CCCCCC");
//         ui_.camera_status->setMovie(&movie);
//         ROS_INFO("DDDDDD");
//         movie.start();
//     }
  
  
  QPixmap pixmap_check;
  pixmap_check.load(check_path);
  int w = ui_.check_status->width();
  int h = ui_.check_status->height();
  ui_.check_status->setPixmap(pixmap_check.scaled(w,h,Qt::KeepAspectRatio));

  QPixmap pixmap_crossmark;
  pixmap_crossmark.load(crossmark_path);
  w = ui_.crossmark_status->width();
  h = ui_.crossmark_status->height();
  ui_.crossmark_status->setPixmap(pixmap_crossmark.scaled(w,h,Qt::KeepAspectRatio));
  QMovie movie_crossmark(crossmark_path);
  ui_.crossmark_status->setMovie(&movie_crossmark);
  movie_crossmark.setSpeed(250);
  movie_crossmark.start();
}

} // namespace dasom_control_gui