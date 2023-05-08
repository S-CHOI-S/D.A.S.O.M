/*
  Copyright 2018
*/
#ifndef RQT_MYPKG_CPP_MY_PLUGIN_H
#define RQT_MYPKG_CPP_MY_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <iostream>
#include "rqt_mypkg/ui_my_plugin.h"
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "std_srvs/Empty.h"
//#include "rqt_mypkg/DasomDynamixel.h"


//#include "rqt_mypkg/FAC_HoverService.h"
//#include <QKeyEvent> 
namespace rqt_mypkg_cpp
{

class MyPlugin : public rqt_gui_cpp::Plugin
{    
    Q_OBJECT
public:
    MyPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();

    template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


private slots:

   // void Arm_Callback(bool val);    
    void publisher_set(const ros::TimerEvent&);
    void callback_set(const ros::TimerEvent&);
//    void TextBox_callback(const ros::TimerEvent&);    
    void ping_callback(const ros::TimerEvent&);    
    void qsc_x_callback(int val);
    void qsc_y_callback(int val);
    void qsc_z_callback(int val);
    void writeLog(QString str);
    void btn_Start_Callback(bool val);
    void AngleSubscriber_Callback(const sensor_msgs::JointState &msg);
    //void LimitSubscriber_Callback(const rqt_mypkg::DasomDynamixel &msg);


    static Eigen::Matrix4d DH(double alpha, double a, double d, double theta)
{
    double cosT = cos(theta), sinT = sin(theta);
    double cosA = cos(alpha), sinA = sin(alpha);
    Eigen::Matrix4d T;
    T <<      cosT, -sinT*cosA,  sinT*sinA,  a*cosT,
              sinT,  cosT*cosA, -cosT*sinA,  a*sinT,
                 0,       sinA,       cosA,       d,
                 0,          0,          0,       1;
    return T;
};


 //   bool FAC_Hover_Callback(rqt_mypkg::FAC_HoverService::Request &req, rqt_mypkg::FAC_HoverService::Response &res);
 //   void keyPressEvent(QKeyEvent *event); 
    
    private:
    Ui::MyPluginWidget ui_;
    QWidget* widget_;
    ros::Publisher publisher;         //이건 GUI Shutdown 용이라서 건들면 안 됨.
    ros::Publisher cmd_Publisher;
    ros::Publisher pub;
    ros::Publisher Test_Pub;
    ros::Subscriber AngleSubscriber;
    ros::Subscriber limitsubscriber;
//    ros::ServiceClient ping_client;
    ros::Timer Publisher_set;
    ros::Timer Callback_set;
    ros::Timer TextBox_set;



    geometry_msgs::Twist test;
    //    ros::ServiceServer HoverServer;
};

}  // namespace rqt_mypkg_cpp

#endif  // RQT_MYPKG_CPP_MY_PLUGIN_H
