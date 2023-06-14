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


#include "test/torque_jacobian.h"

  double time_loop = 0;
  double time_f = 0;
  double time_i = 0;
  double measured_effort = 0;
  int i = 0;
  double l1 = 0.12409;
  double l2 = 0.108;

TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  position_p_gain = node_handle_.param<float>("position_p_gain",1);
  position_i_gain = node_handle_.param<float>("position_i_gain",1);
  position_d_gain = node_handle_.param<float>("position_d_gain",1);
  Cut_Off_Freq = node_handle_.param<float>("Cut_Off_Freq", 1);


  Y_P_gain = node_handle_.param<float>("Y_P_gain",1);
  Y_I_gain = node_handle_.param<float>("Y_I_gain",1);
  Y_D_gain = node_handle_.param<float>("Y_D_gain",1);

  VX_P_gain = node_handle_.param<float>("VX_P_gain",1);
  VX_I_gain = node_handle_.param<float>("VX_I_gain",1);
  VX_D_gain = node_handle_.param<float>("VX_D_gain",1);

  VY_P_gain = node_handle_.param<float>("VY_P_gain",1);
  VY_I_gain = node_handle_.param<float>("VY_I_gain",1);
  VY_D_gain = node_handle_.param<float>("VY_D_gain",1);  
  Error_Gain = node_handle_.param<float>("Error_Gain", 1);
  cut_off_freq_4th = node_handle_.param<float>("cut_off_freq_4th", 1);
  damping_const = node_handle_.param<float>("damping_const", 1);

  stiction_alpha = node_handle_.param<float>("stiction_alpha", 1);
  stiction_k = node_handle_.param<float>("stiction_k", 1);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

// Position_P_gain << X_P_gain, Y_P_gain;
// Position_I_gain << X_I_gain, Y_I_gain;
// Position_D_gain << X_D_gain, Y_D_gain;

// Velocity_P_gain << VX_P_gain, VY_P_gain;
// Velocity_I_gain << VX_I_gain, VY_I_gain;
// Velocity_D_gain << VX_D_gain, VY_D_gain;

  // std::cout<<V_gain<<std::endl<<"---------------------------------------"<<std::endl;

  ROS_INFO("TorqJ node start");



  angle_ref << 0, 0;
  X_cmd << 0, 0;



  // Q_M.resize(4);
  // Q_M_dot.resize(4);
  // Q_M_2.resize(4);
  // Q_M_dot_2.resize(4);  
  // Q_M_A.resize(4, 4);
  // Q_M_B.resize(4, 1);
  // Q_M_C.resize(4);

  Cut_Off_Freq2 = Cut_Off_Freq * Cut_Off_Freq;

   Q_M << 0, 0, 0, 0;
   Q_M_2 << 0, 0, 0, 0;

   Q_M_dot << 0, 0, 0, 0;
   Q_M_dot_2 << 0, 0, 0, 0;

   Q_M_A << 0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1,
           - position_i_gain * Cut_Off_Freq2 / position_d_gain, 
           - (position_p_gain * Cut_Off_Freq2 + sqrt(2) * position_i_gain * Cut_Off_Freq) / position_d_gain, 
           - (position_d_gain * Cut_Off_Freq2 + sqrt(2) * position_p_gain * Cut_Off_Freq + position_i_gain) / position_d_gain, 
           - (position_p_gain + sqrt(2) * position_d_gain * Cut_Off_Freq) / position_d_gain;

   Q_M_B << 0, 0, 0, 1;

   Q_M_C << Cut_Off_Freq2* position_i_gain / position_d_gain, 
            Cut_Off_Freq2* position_p_gain / position_d_gain, 
            Cut_Off_Freq2, 
            Cut_Off_Freq2* polar_moment_1 / position_d_gain;

  Q_M_C = Q_M_C.transpose(); // 1 X 4

  // Q_angle_d.resize(2);
  // Q_angle_d_dot.resize(2);
  // Q_angle_d_2.resize(2);
  // Q_angle_d_dot_2.resize(2);  
  // Q_angle_d_A.resize(2, 2);
  // Q_angle_d_B.resize(2);
  // Q_angle_d_C.resize(2);



  Q_angle_d << 0, 0;
  Q_angle_d_2 << 0, 0;
  Q_angle_d_dot << 0, 0;
  Q_angle_d_dot_2 << 0, 0;
  Q_angle_d_A << 0, 1,
               -Cut_Off_Freq2, -sqrt(2) * Cut_Off_Freq;

  Q_angle_d_B << 0, Cut_Off_Freq2;

  Q_angle_d_C << 1, 0;

  Q_angle_d_C = Q_angle_d_C.transpose(); // 1 X 2

  // Q_M = Q_M.transpose();
  // Q_angle_d = Q_angle_d.transpose();
  // Q_M_2 = Q_M.transpose();
  // Q_angle_d_2 = Q_angle_d.transpose();

}

TorqJ::~TorqJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void TorqJ::initPublisher()
{
  joint_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
  joint_measured_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/measured_dynamixel_position", 10);
  // joint_command_pub = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
}

void TorqJ::initSubscriber()
{
  EE_command_sub_ = node_handle_.subscribe("/goal_EE_position", 10, &TorqJ::commandCallback, this);
  forwardkinematics_sub_ = node_handle_.subscribe("/EE_pose", 10, &TorqJ::poseCallback, this);
  joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &TorqJ::jointCallback, this);
}

void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // X_command, Y_command 값 받아오기(O)
//  X_cmd[0] = msg->position.at(0);
  X_cmd[0] = msg->position.at(0);
  X_cmd[1] = msg->position.at(1);

}

void TorqJ::poseCallback(const geometry_msgs::Twist::ConstPtr &msg)
// 현재 EE position 값 받아오기
{
  X_measured[0] = Measured_EE_Position.linear.x;
  X_measured[1] = Measured_EE_Position.linear.y;

}

void TorqJ::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);

  measured_effort = msg->effort.at(0);
  velocity_measured[0] = msg->velocity.at(0);
  // Jacobian 계산하기()
  J = Jacobian(angle_measured[0], angle_measured[1]);
  // Jacobian transpose()
  JT = J.transpose();
  ///////////////////////////////////////////////////////////////////////
}


void TorqJ::calc_des()
{


//-------커맨드 생성기------//
  // i++;
  // double Amp = 0.05;
  // double period = 8;
  // X_cmd[0] = Amp * (sin(2* M_PI * 0.005 * i / period) + 1);
  // X_cmd[1] = 0.2;
//--------------------------------------

  JT.resize(2,2);


//-----Inverse Kinematics-----//  
  angle_ref[1] = acos((pow(X_cmd[0],2) + pow(X_cmd[1], 2) - pow(l1, 2)- pow(l2, 2)) / (2 * l1 * l2));
  angle_ref[0] = atan(X_cmd[1] / X_cmd[0]) - atan(l2* sin(angle_ref[1]) / (l1 + l2 * cos(angle_ref[1])));
  FK_EE_pos = EE_pos(angle_measured[0], angle_measured[1]);

//  std::cout<<angle_measured<<std::endl<<"angle_measuredangle_measured"<<std::endl;
//  std::cout<<angle_ref<<std::endl<<"angle_refangle_refangle_ref"<<std::endl;
//  std::cout<<X_cmd - FK_EE_pos<<std::endl<<"POSITIONERROR_POSITIONERROR"<<std::endl;
  DoB();
}

void TorqJ::DoB()
{
  // 1번 motor
  //State space (Gn)//
  Q_M_dot = Q_M_A * Q_M + Q_M_B * angle_measured[0];
  Q_M += Q_M_dot * time_loop;
  angle_d_hat[0] = Q_M_C.dot(Q_M); // 여기까지 잘 나오는 거 확인
  

  //State space(theta_d) //
  Q_angle_d_dot = Q_angle_d_A * Q_angle_d + Q_angle_d_B * angle_d[0]; // 13:40 Q_angle_d, angle_d가 nan
  Q_angle_d += Q_angle_d_dot * time_loop;
  angle_d_lpf[0] = Q_angle_d_C.dot(Q_angle_d);


  d_hat[0] = angle_d_hat[0] - angle_d_lpf[0]; //d_hat: 추정된 외란 
    // 14:19 angle_d_hat 잘 나옴 // 14:20 angle_d_lpf가 nan
  angle_d[0] = angle_ref[0] - d_hat[0]; // 14:17 d_hat이 nan

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // 2번 motor
  //State space (Gn)//
  Q_M_dot_2 = Q_M_A * Q_M_2 + Q_M_B * angle_measured[1];
  Q_M_2 += Q_M_dot_2 * time_loop;
  angle_d_hat[1] = Q_M_C.dot(Q_M_2); // 여기까지 잘 나오는 거 확인
  

  //State space(theta_d) //
  Q_angle_d_dot_2 = Q_angle_d_A * Q_angle_d_2 + Q_angle_d_B * angle_d[1]; // 13:40 Q_angle_d, angle_d가 nan
  Q_angle_d_2 += Q_angle_d_dot_2 * time_loop;
  angle_d_lpf[1] = Q_angle_d_C.dot(Q_angle_d_2);


  d_hat[1] = angle_d_hat[1] - angle_d_lpf[1]; //d_hat: 추정된 외란 
    // 14:19 angle_d_hat 잘 나옴 // 14:20 angle_d_lpf가 nan
  angle_d[1] = angle_ref[1] - d_hat[1]; // 14:17 d_hat이 nan


//   for (int i =0; i<2; i++)
// {
//   //State space (Gn)//
//   Q_M_dot = Q_M_A * Q_M + Q_M_B * angle_measured[i];
//   Q_M += Q_M_dot * time_loop;
//   angle_d_hat[i] = Q_M_C.dot(Q_M); // 여기까지 잘 나오는 거 확인
  

//   //State space(theta_d) //
//   Q_angle_d_dot = Q_angle_d_A * Q_angle_d + Q_angle_d_B * angle_d[i]; // 13:40 Q_angle_d, angle_d가 nan
//   Q_angle_d += Q_angle_d_dot * time_loop;
//   angle_d_lpf[i] = Q_angle_d_C.dot(Q_angle_d);


//   d_hat[i] = angle_d_hat[i] - angle_d_lpf[i]; //d_hat: 추정된 외란 
//     // 14:19 angle_d_hat 잘 나옴 // 14:20 angle_d_lpf가 nan
//   angle_d[i] = angle_ref[i] - d_hat[i]; // 14:17 d_hat이 nan
// }

  std::cout<<time_loop<<std::endl<<"time_loop"<<std::endl;
  std::cout<<Q_angle_d<<std::endl<<"Q_angle_d"<<std::endl;
  std::cout<<angle_d<<std::endl<<"angle_d"<<std::endl;
  // std::cout<<angle_d_lpf<<std::endl<<"angle_d_lpfangle_d_lpf"<<std::endl;
  // std::cout<<angle_d<<std::endl<<"angle_dangle_dangle_d"<<std::endl;

  // Eigen::

}
/*  void TorqJ::second_order_butterworth()
{ 
  
  wc = tan(M_PI * cut_off_freq * time_loop);
  wc2 = wc*wc;

  b0_2nd = (wc2) / (1 + sqrt(2)*wc + wc2);
  b1_2nd = 2 * b0_2nd;
  b2_2nd = b0_2nd;
  a0_2nd = 1;
  a1_2nd = 2 * (wc2 - 1) / (1 + sqrt(2) * wc + wc2);
  a2_2nd = (1 - sqrt(2) * wc + wc2) / (1 + sqrt(2) * wc + wc2);


  bw_2nd_input[2] = X_error_p[0];

  bw_2nd_output[2] = b0_2nd * bw_2nd_input[2] + b1_2nd * bw_2nd_input[1] + b2_2nd * bw_2nd_input[0] - a1_2nd * bw_2nd_output[1] - a2_2nd * bw_2nd_output[0];

  bw_2nd_output[0] = bw_2nd_output[1];
  bw_2nd_output[1] = bw_2nd_output[2];

  bw_2nd_input[0] = bw_2nd_input[1];
  bw_2nd_input[1] = bw_2nd_input[2];
  //---------------------------------------


}
*/
/*
  void TorqJ::stiction_gravity_compensator()
{
  //stiction_gain[0] = stiction_k * X_error_p[0] / exp(stiction_alpha * velocity_measured[0]);    //exponential

  stiction_gain[0] = stiction_k * X_error_p[0] * (1 - pow(tanh(stiction_alpha * velocity_measured[0]), 2));
  
  double torque_const = 0.5;
	// 중력 매트릭스 (1링크에다가 OCM) onelink
	tau_gravity[0] = torque_const * mass2 * CoM2 * cos(angle_measured[0]) * 9.81;

  std::cout<<stiction_gain[0]<<"tau_gravitau_gravitau_gravitau_gravitau_gravitau_gravitau_gravitau_gravitau_gravi"<<std::endl;

}
*/
/*
  void TorqJ::second_order_butterworth_()
{ 
  
  wc_4th = tan(M_PI * cut_off_freq_4th * time_loop);
  wc2_4th = wc_4th*wc_4th;

  b0_4th = (wc2) / (1 + sqrt(2)*wc + wc2);
  b1_4th = 2 * b0_4th;
  b2_4th = b0_4th;
  a0_4th = 1;
  a1_4th = 2 * (wc2 - 1) / (1 + sqrt(2) * wc + wc2);
  a2_4th = (1 - sqrt(2) * wc + wc2) / (1 + sqrt(2) * wc + wc2);


  bw_4th_input[2] = velocity_measured[0];

  bw_4th_output[2] = b0_4th * bw_4th_input[2] + b1_4th * bw_4th_input[1] + b2_4th * bw_4th_input[0] - a1_4th * bw_4th_output[1] - a2_4th * bw_4th_output[0];
  velocity_filtered[0] = bw_4th_output[2];
  bw_4th_output[0] = bw_4th_output[1];
  bw_4th_output[1] = bw_4th_output[2];

  bw_4th_input[0] = bw_4th_input[1];
  bw_4th_input[1] = bw_4th_input[2];

  //---------------------------------------

}
*/

/*
int k=0;
  void TorqJ::friction_compen_pulse()
{  
  k_fc = 1.0;
  Delta = 0.1;
  Cut_Off_Freq = 5.0;

  if(abs(u_fc[0])<Delta) 
  {
    w0 = X_error_p[0] * k_fc;
    k = 0;
  }

  k++;
  Pd = exp(-2*M_PI*Cut_Off_Freq*time_loop);
  u_fc[0] = ( 1 + Pd ) * pow(Pd,k) * w0;

}
*/


void TorqJ::calc_taudes()
{
//  tau_des = tau_loop + tau_gravity + u_fc;    //  펄스
  tau_des = tau_loop + tau_gravity + stiction_gain - damping_const * velocity_measured;              //  로우패스필터



  ROS_WARN("update!");
}

void TorqJ::PublishCmdNMeasured()
{
  sensor_msgs::JointState joint_cmd;
  sensor_msgs::JointState joint_measured;

  // tau_des를 publish
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.position.push_back(angle_ref[0]); // 커맨드포지션
  joint_cmd.position.push_back(angle_ref[1]); // 측정포지션
  joint_cmd.velocity.push_back(X_error_p[0]); // 포지션 에러
  joint_cmd.velocity.push_back(bw_2nd_output[2]); // 포지션 에러 필터값
  joint_cmd.effort.push_back(tau_des[0]); // joint space
  joint_cmd.effort.push_back(tau_des[1]); // joint space
  joint_command_pub_.publish(joint_cmd);

  joint_measured.header.stamp = ros::Time::now();
  joint_measured.position.push_back(angle_ref[0]); // 포지션 에러값
  joint_measured.position.push_back(angle_d[0]); // 포지션 에러 필터값 
  joint_measured.velocity.push_back(tau_des[0] + stiction_gain[0]); // 
  joint_measured.velocity.push_back(u_fc[0] + tau_des[0]);
  joint_measured_pub_.publish(joint_measured);
}


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(200); //250


	time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
    time_f = ros::Time::now().toSec();
		time_loop = time_f - time_i;
		time_i = ros::Time::now().toSec();

    torqJ.calc_des();
    torqJ.calc_taudes();
    torqJ.PublishCmdNMeasured();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}