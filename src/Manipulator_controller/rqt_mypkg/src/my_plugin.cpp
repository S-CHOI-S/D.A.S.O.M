/*
  Copyright 2018
*/
#include "my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

double PI = 3.141592;
bool isPublishing = false;
bool isCallback = false;
float value_x = 0;
float value_y = 0;
float value_z = 0;

float cmd_x = 0;
float cmd_y = 0;
float cmd_z = 0;

double l1 = 0.10375;
double l2 = 0.153;
double l3 = 0.12409;
double l4 = 0.14909;

double FK_x = 0;
double FK_y = 0;
double FK_z = 0;

double q1 = 0;
double q2 = 0;
double q3 = 0;
double q4 = 0;

int Flag_1 = 0;
int Flag_2 = 0;
int Flag_3 = 0;
int Flag_4 = 0;


double Request_Send = 0;
double Response_Receive = 0;

namespace rqt_mypkg_cpp
{



MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("C++PluginT");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);
////////////////////////////////////////////////////////////////////////////////////////
  ros::start();
  ros::NodeHandle n;


    QObject::connect(ui_.qsc_x, SIGNAL(valueChanged(int)), this, SLOT(qsc_x_callback(int)));
    QObject::connect(ui_.qsc_y, SIGNAL(valueChanged(int)), this, SLOT(qsc_y_callback(int)));
    QObject::connect(ui_.qsc_z, SIGNAL(valueChanged(int)), this, SLOT(qsc_z_callback(int)));
    QObject::connect(ui_.btn_Start, SIGNAL(toggled(bool)), this, SLOT(btn_Start_Callback(bool))); // btn_Read_Topic_Callback (Normal)
//  QObject::connect(ui_.btn_Manipulator, SIGNAL(clicked(bool)), this, SLOT(Manipulator_Callback(bool))); // Manipulator button (Click)


  Publisher_set = n.createTimer(ros::Duration(0.004), &MyPlugin::publisher_set, this);
  Callback_set = n.createTimer(ros::Duration(0.1), &MyPlugin::callback_set, this);
//  TextBox_set = n.createTimer(ros::Duration(0.1), &MyPlugin::TextBox_callback, this);

  //HoverServer = n.advertiseService("/FAC_HoverService", &MyPlugin::FAC_Hover_Callback, this); // Get state from Drone to GUI
  // cmd_Publisher = n.advertise<sensor_msgs::JointState>("/reference_position", 100);  // Dynamixel에 direct로 cmd 값 주기
cmd_Publisher = n.advertise<sensor_msgs::JointState>("/goal_EE_position", 100);  // Dynamixel에 direct로 cmd 값 주기
//  cmd_Publisher = n.advertise<geometry_msgs::Twist>("/goal_EE_position", 100);
  AngleSubscriber = n.subscribe("/joint_states", 100, &MyPlugin::AngleSubscriber_Callback, this);
 // limitsubscriber = n.subscribe("/dasom/goal_dynamixel_position", 100, &MyPlugin::LimitSubscriber_Callback, this);
//  ping_client = n.serviceClient<std_srvs::Empty>("/ping_tester");
  Test_Pub = n.advertise<geometry_msgs::Twist>("/Test", 100);









}




void MyPlugin::shutdownPlugin()
{

    publisher.shutdown();
    cmd_Publisher.shutdown();
    AngleSubscriber.shutdown();
    ros::shutdown();
}


void MyPlugin::writeLog(QString str)
{
  ui_.pte_stateLog->moveCursor (QTextCursor::End);
  ui_.pte_stateLog->appendPlainText(str);
}



void MyPlugin::callback_set(const ros::TimerEvent&) 
{
  if(isCallback)
  {
      ui_.txt_joint1->setText((QString::number(q1)));
      ui_.txt_joint2->setText((QString::number(q2)));  

	double Link1 = 0.10375;
	double Link2 = 0.153;

    Eigen::Vector2d FK;
    FK <<   Link1*cos(q1) + Link2 * cos(q1+q2),
        Link1*sin(q1) + Link2 * sin(q1+q2);
      
      ui_.txt_FK_x->setText((QString::number(FK[0])));
      ui_.txt_FK_y->setText((QString::number(FK[1])));


    ui_.txt_Error_x->setText((QString::number(value_x - FK[0], 'f', 3)));
    ui_.txt_Error_y->setText((QString::number(value_y - FK[1], 'f', 3)));  


          if(! ui_.chk_Publish->isChecked() && isCallback)
          {
 //         ui_.qsc_x->setValue(FK[0] * 10000000);   //현재 ENd EFfector의 위치를 넣자.
 //         ui_.qsc_y->setValue(FK[1] * 10000000);
          }
  }   
  /*
  if (isCallback) 
  {
    ui_.txt_joint1->setText((QString::number(q1)));
    ui_.txt_joint2->setText((QString::number(q2)));
    ui_.txt_joint3->setText((QString::number(q3)));
    ui_.txt_joint4->setText((QString::number(q4)));


    Eigen::Matrix4d T =
          DH(  EIGEN_PI/2 ,   0,  l1, q1) *
          DH(       0     ,  l2,   0, q2) *
          DH(       0     ,  l3,   0, q3) *
          DH( - EIGEN_PI/2,  l4,   0, q4);

      FK_x = T.coeff(0,3);
      FK_y = T.coeff(1,3);
      FK_z = T.coeff(2,3);


    ui_.txt_FK_x->setText((QString::number(FK_x)));
    ui_.txt_FK_y->setText((QString::number(FK_y)));
    ui_.txt_FK_z->setText((QString::number(FK_z)));



    ui_.txt_Error_x->setText((QString::number(value_x - FK_x)));
    ui_.txt_Error_y->setText((QString::number(value_y - FK_y)));  
    ui_.txt_Error_z->setText((QString::number(value_z - FK_z)));
  

      // 안전장치 (QScroll Bar) //
        if(! ui_.chk_Publish->isChecked() && isCallback)
        {
        ui_.qsc_x->setValue(FK_x * 10000000);   //현재 ENd EFfector의 위치를 넣자.
        ui_.qsc_y->setValue(FK_y * 10000000);
        ui_.qsc_z->setValue(FK_z * 10000000);
        
        }

   ros::spinOnce();
  }
*/}


void MyPlugin::publisher_set(const ros::TimerEvent&) 
{
  if(ui_.chk_Publish->isChecked() && isCallback) 
  {
//  geometry_msgs::Twist cmd_Position;
//  cmd_Position.linear.x = value_x;
//  cmd_Position.linear.y = value_y;
//  cmd_Position.linear.z = value_z;

  sensor_msgs::JointState msg;


  ui_.lbl_cmd_x->setText((QString::number(value_x, 'f', 5)));
  ui_.lbl_cmd_y->setText((QString::number(value_y, 'f', 5)));
  ui_.lbl_cmd_z->setText((QString::number(value_z, 'f', 5)));


  test.linear.x = value_x;
  test.linear.y = value_y;
  test.linear.z = value_z;


  msg.position.push_back(value_x);
  msg.position.push_back(value_y);

  cmd_Publisher.publish(msg);
  }

}


void MyPlugin::qsc_x_callback(int val)
{
  sensor_msgs::JointState msg;

  value_x = val;
	value_x = map<double>(value_x, 0, 10000000, 0,0.3);
}

void MyPlugin::qsc_y_callback(int val)
{
  value_y = val;
	value_y = map<double>(value_y, 0, 10000000, 0,0.3);

}

void MyPlugin::qsc_z_callback(int val)
{
  value_z = val;

}

//  ui_.lbl_cmd_z->setText((QString::number(value_z, 'f', 3)));


void MyPlugin::btn_Start_Callback(bool val)
{
  if(ui_.btn_Start->isChecked()) 
  {
  isCallback = true;
  ui_.btn_Start->setText("Starting..");
  }
  else 
  {
  isCallback = false;
  ui_.btn_Start->setText("Start!");

  ui_.txt_joint1->setText("0");
  ui_.txt_joint2->setText("0");
  ui_.txt_joint3->setText("0");

  ui_.txt_FK_x->setText("0");
  ui_.txt_FK_y->setText("0");
  ui_.txt_FK_z->setText("0");

  ui_.txt_Error_x->setText("0");
  ui_.txt_Error_y->setText("0");
  ui_.txt_Error_z->setText("0");

  }
  return ;
}




void MyPlugin::AngleSubscriber_Callback(const sensor_msgs::JointState &msg)
{
  q1 = msg.position.at(0);
  q2 = msg.position.at(1);
//  q3 = msg.position.at(2);
//  q4 = msg.position.at(3);

//  Flag_1 = msg.flag.at(0);
//  Flag_2 = msg.flag.at(1);
//  Flag_3 = msg.flag.at(2);

}

// void MyPlugin::LimitSubscriber_Callback(const rqt_mypkg::DasomDynamixel &msg)
// {
//   Flag_1 = msg.flag.at(0);
//   Flag_2 = msg.flag.at(1);
// // Flag_3 = msg.flag.at(2);
// //  Flag_4 = msg.flag.at(3);
  
// }



/*
void MyPlugin::Kill_Callback(bool val)
{
    rqt_mypkg::KillService Service;
    if(ui_.btn_Kill->isChecked())
    {
      Service.request.Kill_isChecked = true;

        if (KillClient.call(Service))
        {
        ui_.btn_Kill->setText("Killed");
        ui_.btn_Kill->setStyleSheet("background-color:none");        
        }
        else 
        {
          ui_.btn_Kill->setText("Kill Failed");
          ui_.btn_Kill->setStyleSheet("background-color:red");
        }
    }
    else 
    {
      Service.request.Kill_isChecked = false;
      ui_.btn_Kill->setText("Kill");
      ui_.btn_Kill->setStyleSheet("background-color:none");
    }
    KillClient.call(Service);
    ros::spinOnce();
}


void MyPlugin::Hover_Callback(bool val)
{   
  if(isArm)
  {
      rqt_mypkg::HoverService Service;
      ui_.edi_Alti->setText("100");
      
      Service.request.isHover = true;
      Service.request.isHovering = true;
      Service.request.isLanding = false;

          if(HoverClient.call(Service))
          {
          isHover = true;
          ui_.btn_Hover->setText("Hovering");
          ui_.btn_Hover->setStyleSheet("background-color:Yellow");
          ui_.btn_Land->setText("Land");
          ui_.btn_Land->setStyleSheet("background-color:None");
          }
          else
          {
          ui_.btn_Hover->setText("Failed Hover");
          ui_.btn_Hover->setStyleSheet("background-color:Red");
          }



      HoverClient.call(Service);
      ros::spinOnce();
  }
}


void MyPlugin::Land_Callback(bool val)
{
  if(isHover)
  {
      rqt_mypkg::HoverService Service;
      ui_.edi_Alti->setText("0");
      Service.request.isHover = true;
      Service.request.isLanding = true;
      Service.request.isHovering = false;

          if(HoverClient.call(Service))
          {
          isHover = false;
          isArm = false;
          ui_.btn_Hover->setText("Hover");
          ui_.btn_Hover->setStyleSheet("background-color:None");
          ui_.btn_Arm->setText("Arm");
          ui_.btn_Arm->setStyleSheet("background-color:None");
          }
          else
          {
          ui_.btn_Land->setText("Failed Land");
          ui_.btn_Land->setStyleSheet("background-color:Red");
          }


      HoverClient.call(Service);
      ros::spinOnce();
  }
}


void MyPlugin::PosCtrl_Callback(bool val)
{
  if(isHover)
  {
  rqt_mypkg::PosCtrlService Service;


  Service.request.desired_X = ui_.edi_X->text().toDouble() / 100;
  Service.request.desired_Y = ui_.edi_Y->text().toDouble() / 100;
  Service.request.desired_Yaw = ui_.edi_Yaw->text().toDouble() * PI / 180.0; 
  Service.request.desired_Alti = ui_.edi_Alti->text().toDouble() / 100;


      if(PosCtrlClient.call(Service))
      {
      ui_.btn_PosCtrl->setText("Position Ctrl");
      ui_.btn_PosCtrl->setStyleSheet("background-color:none");
      }
      else
      {
      ui_.btn_PosCtrl->setText("Failed Ctrl");
      ui_.btn_PosCtrl->setStyleSheet("background-color:Red");
      }

  PosCtrlClient.call(Service);
  ros::spinOnce();
  }
}


void MyPlugin::Tilt_Callback(bool val)
{

    rqt_mypkg::TiltService Service;
    if(ui_.btn_Tilt->isChecked())
    {
      Service.request.Tilt_isChecked = true;

        if (TiltClient.call(Service))
        {
        ui_.btn_Tilt->setText("Tilted");
        ui_.btn_Tilt->setStyleSheet("background-color:Green");        
        }
        else 
        {
          ui_.btn_Tilt->setText("Tilt Failed");
          ui_.btn_Tilt->setStyleSheet("background-color:red");
        }
    }
    else 
    {
      Service.request.Tilt_isChecked = false;
      ui_.btn_Tilt->setText("Tilt");
      ui_.btn_Tilt->setStyleSheet("background-color:none");
    }
    TiltClient.call(Service);
    ros::spinOnce();

}



//------------------------Docking-----------------------//



void MyPlugin::Dock_Callback(bool val)
{
    rqt_mypkg::DockService Service;
      

          if(ui_.btn_Dock->isChecked())
          {
          Service.request.Dock_Do = true;
          ui_.btn_Dock->setText("Docking");
          ui_.btn_Dock->setStyleSheet("background-color:Green");
          }
          else
          { 
          Service.request.Dock_Do=false;
          ui_.btn_Dock->setText("Dock");
          ui_.btn_Dock->setStyleSheet("background-color:None");
          }

      DockClient.call(Service);
      ros::spinOnce();
}


void MyPlugin::Switch_callback(const std_msgs::UInt16 &isdock)
{

  if(isdock.data == 0) ui_.lbl_Dock->setText("Docking");
  else ui_.lbl_Dock->setText("UnDocking");

}


//-----------------------Battery Change with Manipulator---------------------------//
void MyPlugin::Manipulator_Callback(bool val)
{
    rqt_mypkg::ManipulatorService Service;
    Service.request.Manipul = true;
   
    ManipulatorClient.call(Service);
    ros::spinOnce();
    return ;
}












void MyPlugin::run() {
  ros::Rate loop_rate(10); //일단주파수는 10정도로 해놓자
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}


//------------------------------Drone to GUI callback. --------------------//
bool MyPlugin::FAC_Hover_Callback(rqt_mypkg::FAC_HoverService::Request &req, 
                                  rqt_mypkg::FAC_HoverService::Response &res){  //ASDF
isHover = req.FAC_isHover;
isHovering = req.FAC_isHovering;
isLanding = req.FAC_isLanding;

  if(!req.FAC_isHovering && req.FAC_isHover)  // 드론에서 Hover 했다고 신호가 옴
  {
    ui_.btn_Hover->setText("Hovered");
    ui_.btn_Hover->setStyleSheet("background-color:Green");
    ui_.btn_PosCtrl->setStyleSheet("background-color:Green");
  }

  if(!req.FAC_isLanding && !req.FAC_isHover)  //드론에서 Land 했다고 신호가 옴
  {
    ui_.btn_Land->setText("Land");
    ui_.btn_Land->setStyleSheet("background-color:None");

    isArm = false;
    isHovering = false;
    ui_.btn_Arm->setText("Arm");
    ui_.btn_Arm->setStyleSheet("background-color:None");
  }
return true;
}

//------------------------------스페이스바를 누르면 Kill 상태로 보낼 수 있게 만들려고 했는데 일단 실패함 --------------------//
void MyPlugin::keyPressEvent(QKeyEvent *event)
{
  if(event->key() == Qt::Key_Space)
  { 
  ui_.btn_Kill->isChecked();
  }
}


//------------------------------아마도 군집 조종하기 위해 사용할 것 같음.... --------------------//
void MyPlugin::gotogether(bool val)
{
  secondturtles::turtlesrv server;

    if(ui_.chk_t1->isChecked())
    {
    server.request.a = ui_.lbl_a->text().toDouble();
    server.request.b = ui_.lbl_b->text().toDouble(); 
    servicecaller.call(server);   
    ros::spinOnce();    
    }

    if(ui_.chk_t2->isChecked())
    {
    server.request.c = ui_.lbl_c->text().toDouble();
    server.request.d = ui_.lbl_d->text().toDouble();    
    secondservicecaller.call(server);   
    ros::spinOnce();
    }

}


*/

}  // namespace rqt_mypkg_cpp









// #define PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)
PLUGINLIB_EXPORT_CLASS(rqt_mypkg_cpp::MyPlugin, rqt_gui_cpp::Plugin)


