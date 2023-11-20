/*******************************************************************************
* D.A.S.O.M
*
* Department of Aerial Manipulator System for Object Manipulation
*
*     https://github.com/S-CHOI-S/D.A.S.O.M.git
*
* Mobile Robotics Lab. (MRL)
* 	  @ Seoul National University of Science and Technology
*
*	  https://mrl.seoultech.ac.kr/index.do
*
*******************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <dasom_controllers/haptic_inst.h>

#define PI 3.14159256359

HI::HI()
: node_handle_(""),
  priv_node_handle_("~")
{

    ds_wb_ = new dasom::DasomWorkbench;

  // ds_wb_->run();

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/

  initSubscriber();

  X_test.resize(6,1);
  X_cmd.resize(6,1);
  X_ref.resize(6,1);
  angle_ref.resize(6,1);
  angle_ref_i.resize(6,1); // ADD
  angle_d.resize(6,1);
  FK_pose.resize(6,1);
  haptic_command.resize(6,1);
  haptic_initPose.resize(6,1);
  initPose.resize(6, 1);

  angle_measured.resize(6,1);
  tau_measured.resize(6,1);
  firstPose.resize(6,1);
  
  J.resize(6,6);
  JT.resize(6,6);
  JTI.resize(6,6);
  PoseDelta.resize(6,1);

  hysteresis_max.resize(6,1);
  hysteresis_min.resize(6,1);

  angle_max.resize(6,1);
  angle_min.resize(6,1);
  angle_safe.resize(6,1);

  angle_ref << 0, 0, 0, 0, 0, 0;
  angle_ref_i << 0, 0, 0, 0, 0, 0;

  Cut_Off_Freq2 = Cut_Off_Freq * Cut_Off_Freq;

  angle_d << 0, 0, 0, 0, 0, 0;
  angle_d_lpf << 0, 0;

  angle_safe << 0, 0, 0, 0, 0, 0;

  //--For Model of DOB--//
  Q_M << 0, 0, 0, 0;
  Q_M_2 << 0, 0, 0, 0;

  Q_M_dot << 0, 0, 0, 0;
  Q_M_dot_2 << 0, 0, 0, 0;

  Q_M_B << 0, 0, 0, 1;


  Q_M_A << 0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
      -position_i_gain * Cut_Off_Freq2 / position_d_gain,
      -(position_p_gain * Cut_Off_Freq2 + sqrt(2) * position_i_gain * Cut_Off_Freq) / position_d_gain,
      -(position_d_gain * Cut_Off_Freq2 + sqrt(2) * position_p_gain * Cut_Off_Freq + position_i_gain) / position_d_gain,
      -(position_p_gain + sqrt(2) * position_d_gain * Cut_Off_Freq) / position_d_gain;

  Q_M_C << Cut_Off_Freq2 * position_i_gain / position_d_gain,
      Cut_Off_Freq2 * position_p_gain / position_d_gain,
      Cut_Off_Freq2,
      Cut_Off_Freq2 * polar_moment_1 / position_d_gain;


  //---For Q-filter--//
  Q_angle_d << 0, 0;
  Q_angle_d_2 << 0, 0;
  Q_angle_d_dot << 0, 0;
  Q_angle_d_dot_2 << 0, 0;
  Q_angle_d_A << 0, 1,
      -Cut_Off_Freq2, -sqrt(2) * Cut_Off_Freq;

  Q_angle_d_B << 0, Cut_Off_Freq2;

  Q_angle_d_C << 1, 0;

  Q_angle_d_C = Q_angle_d_C.transpose();

  //////////////////////////////////////////
  ////////////////Admittance////////////////
  //////////////////////////////////////////
  A_x << 0, 1,
      -virtual_spring[0] / virtual_mass[0], -virtual_damper[0] / virtual_mass[0];

  B_x << 0, 1 / virtual_mass[0];

  C_x << 1, 0;

  D_x << 0, 0;

  A_y << 0, 1,
      -virtual_spring[1] / virtual_mass[1], -virtual_damper[1] / virtual_mass[1];

  B_y << 0, 1 / virtual_mass[1];

  C_y << 1, 0;

  D_y << 0, 0;

  A_z << 0, 1,
      -virtual_spring[2] / virtual_mass[2], -virtual_damper[2] / virtual_mass[2];

  B_z << 0, 1 / virtual_mass[2];

  C_z << 1, 0;

  D_z << 0, 0;


  X_from_model_matrix << 0, 0;
  X_dot_from_model_matrix << 0, 0;

  Y_from_model_matrix << 0, 0;
  Y_dot_from_model_matrix << 0, 0;

  Z_from_model_matrix << 0, 0;
  Z_dot_from_model_matrix << 0, 0;


  //------------------//
  // Tuning parameter //
  //------------------//

  //--Dead Zone--//
  hysteresis_max << 0.35, 0.27, 0.2, 0, 0, 0; //나바
  hysteresis_min << -0.24, -0.35, -0.2, 0, 0, 0;

  //--Angle saturation--//
  // angle_max << 1.6, 1.9, 0.75, 1, 1, 1; //나바
  // angle_min << -0.7, -1.8, -2, -1, -1, -1;
  angle_max << M_PI/2+0.1, M_PI, 0, 2.35, 2.35, 2; //나바
  angle_min << -M_PI/2-0.1, 0, -2.1, -2.35, -2.35, -0.3;

  //--F_ext saturation--//
  Force_max << 2.0, 1.0, 0; //나바
  Force_min << -2.0, -1.0, 0;


  X_test << 0, 0, 0, 0, 0, 0;
  FK_pose << 0, 0, 0, 0, 0, 0;

  initPose << 0, 0.1, 0.35, M_PI/2, 0, 0;
}

HI::~HI()
{

}

void HI::initSubscriber()
{
    joint_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    // joint_measured_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/measured_dynamixel_position", 10);  
    // dasom_EE_pos_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/dasom/EE_pose", 10);  
    dasom_EE_cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/phantom/xyzrpy", 10);
    haptic_button_pub_ = node_handle_.advertise<omni_msgs::OmniButtonEvent>("/phantom/button", 10);
    // joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &HI::jointCallback, this, ros::TransportHints().tcpNoDelay());
    // joystick_sub_ = node_handle_.subscribe("/instead_haptic", 10, &HI::joystickCallback, this, ros::TransportHints().tcpNoDelay());
}

void HI::solveInverseKinematics()
{
    geometry_msgs::Twist msg;
    sensor_msgs::JointState js;

    // EE_position과 orientation이 들어왔을 것: X_ref, Orientation_ref

    angle_ref = InverseKinematics(haptic_command[0], haptic_command[1], haptic_command[2],
                                  haptic_command[3], haptic_command[4], haptic_command[5]);

    for(int i = 3; i < 6; i++)
    {
        if(abs(angle_ref[i] - angle_ref_i[i]) >= 3.14 && abs(angle_ref[i] - angle_ref_i[i]) <= 2 * 3.14)
        {
            if(angle_ref[i] > 0) 
            {
                angle_ref[i] -= PI; 
                // ROS_ERROR("HERE! (+) -> (-)");
            }
            else 
            {
                angle_ref[i] += PI; 
                // ROS_ERROR("THERE! (-) -> (+)");
            }
        }
    }

    // ROS_INFO("=============angle command from inverse kinematics=========");
    // ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", angle_ref[0], angle_ref[1], angle_ref[2], angle_ref[3], angle_ref[4], angle_ref[5]);
    // ROS_INFO("=============angle measured from inverse kinematics=========");
    // ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", angle_measured[0], angle_measured[1], angle_measured[2], angle_measured[3], angle_measured[4], angle_measured[5]);

    FK_pose = EE_pose(angle_ref[0], angle_ref[1], angle_ref[2], angle_ref[3], angle_ref[4], angle_ref[5]);
    
    // ROS_INFO("============error angle================");
    // ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", angle_measured[0] - angle_ref[0], angle_measured[1] - angle_ref[1], angle_measured[2] - angle_ref[2], angle_measured[3] - angle_ref[3], angle_measured[4] - angle_ref[4], angle_measured[5] - angle_ref[5]);
    // ROS_WARN("======================================================================================");
    // ROS_ERROR("%lf, %lf, %lf, %lf, %lf, %lf", angle_measured[0], angle_measured[1], angle_measured[2], angle_measured[3], angle_measured[4], angle_measured[5]);

    msg.linear.x = haptic_command[0];
    msg.linear.y = haptic_command[1];
    msg.linear.z = haptic_command[2];
    msg.angular.x = FK_pose[3];
    msg.angular.y = FK_pose[4];
    msg.angular.z = FK_pose[5];

    js.header.stamp = ros::Time::now();
    js.name.resize(6);
    js.name[0] = "id_1";
    js.name[1] = "id_2";
    js.name[2] = "id_3";
    js.name[3] = "id_4";
    js.name[4] = "id_5";
    js.name[5] = "id_6";
    js.position.push_back(angle_ref[0]);
    js.position.push_back(angle_ref[1]);
    js.position.push_back(angle_ref[2]);
    js.position.push_back(angle_ref[3]);
    js.position.push_back(angle_ref[4]);
    js.position.push_back(angle_ref[5]);

    dasom_EE_cmd_pub_.publish(msg);
    joint_command_pub_.publish(js);

    angle_ref_i = angle_ref;
  
}

int td = 0;
double i = 0;
void HI::CommandGenerator()
{
    geometry_msgs::Twist msg__;

    i++;

    msg__.linear.x =0.1*sin(i/1000);
    msg__.linear.y =0.1*sin(i/1000); // y축 방향으로 안정적인 거 확인!
    msg__.linear.z = 0.1*sin(i/1000); // z축 방향으로 안정적인 거 확인!
    msg__.angular.x = 0;
    msg__.angular.y = 0;
    msg__.angular.z = 0;

    // haptic_cmd_.publish(msg__);

    haptic_command[0] = msg__.linear.x;
    haptic_command[1] = msg__.linear.y;
    haptic_command[2] = msg__.linear.z;
    haptic_command[3] = 0; //msg.angular.x;
    haptic_command[4] = 0; //msg.angular.y;
    haptic_command[5] = 0; //msg.angular.z;  
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // tf::Matrix3x3(quat).getRPY(haptic_command[3], haptic_command[4], haptic_command[5]);
    
    // haptic_command = haptic_command + initPose;

    ROS_WARN("================================");
    ROS_INFO("X_cmd = %lf",haptic_command[0]);
    ROS_INFO("Y_cmd = %lf",haptic_command[1]);
    ROS_INFO("Z_cmd = %lf",haptic_command[2]);

    omni_msgs::OmniButtonEvent button;

    if(td % 3500 == 0)
    {
        button.grey_button = 0;
        button.grey_button = 1;
    }
    
    td++;

    haptic_button_pub_.publish(button);
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "haptic_inst");
    ros::NodeHandle nh;
    ros::Publisher haptic_cmd_ = nh.advertise<geometry_msgs::Twist>("/palletrone/battery",10);
    HI hi;
    ros::Rate loop_rate(250);

    geometry_msgs::Twist msg;
    int i = 0;

    nh.setParam("/haptic_node_started", true);
    ROS_WARN("Haptic Device is running!");   

    while(ros::ok())
    {
        hi.CommandGenerator();
        hi.solveInverseKinematics();

        // msg.linear.x = i%5;
        // msg.linear.y = -0.05; // y축 방향으로 안정적인 거 확인!
        // msg.linear.z = 0; // z축 방향으로 안정적인 거 확인!
        // msg.angular.x = 0;
        // msg.angular.y = 0;
        // msg.angular.z = 0;

        // haptic_cmd_.publish(msg);

        // i++;
        // ROS_INFO("X_cmd = %lf",0);
        // ros::spin();
	    loop_rate.sleep();
    }
    
    return 0;
}