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


#include "test/torque_jacobian_icros.h"

  double time_loop = 0;
  double time_f = 0;
  double time_i = 0;
  int i = 0;
  double l1 = 0.12409;
  double l2 = 0.108;
  double Kt = 0.0044;

TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  position_p_gain = node_handle_.param<double>("position_p_gain",1);
  position_i_gain = node_handle_.param<double>("position_i_gain",1);
  position_d_gain = node_handle_.param<double>("position_d_gain",1);
  Cut_Off_Freq = node_handle_.param<double>("Cut_Off_Freq", 1);


  Y_P_gain = node_handle_.param<double>("Y_P_gain",1);
  Y_I_gain = node_handle_.param<double>("Y_I_gain",1);
  Y_D_gain = node_handle_.param<double>("Y_D_gain",1);

  VX_P_gain = node_handle_.param<double>("VX_P_gain",1);
  VX_I_gain = node_handle_.param<double>("VX_I_gain",1);
  VX_D_gain = node_handle_.param<double>("VX_D_gain",1);

  VY_P_gain = node_handle_.param<double>("VY_P_gain",1);
  VY_I_gain = node_handle_.param<double>("VY_I_gain",1);
  VY_D_gain = node_handle_.param<double>("VY_D_gain",1);  
  Error_Gain = node_handle_.param<double>("Error_Gain", 1);
  cut_off_freq_4th = node_handle_.param<double>("cut_off_freq_4th", 1);
  damping_const = node_handle_.param<double>("damping_const", 1);

  stiction_alpha = node_handle_.param<double>("stiction_alpha", 1);
  stiction_k = node_handle_.param<double>("stiction_k", 1);

  virtual_mass_x = node_handle_.param<double>("virtual_mass_x",1);
  virtual_damper_x = node_handle_.param<double>("virtual_damper_x",1);
  virtual_spring_x = node_handle_.param<double>("virtual_spring_x",1);

  virtual_mass_y = node_handle_.param<double>("virtual_mass_y",1);
  virtual_damper_y = node_handle_.param<double>("virtual_damper_y",1);
  virtual_spring_y = node_handle_.param<double>("virtual_spring_y",1);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  virtual_mass << virtual_mass_x, virtual_mass_y;
  virtual_damper << virtual_damper_x, virtual_damper_y;
  virtual_spring << virtual_spring_x, virtual_spring_y;

  // std::cout<<V_gain<<std::endl<<"---------------------------------------"<<std::endl;

  ROS_INFO("TorqJ node start");



  angle_ref << 0, 0;
  X_ref << 0.2, 0.1;



  // Q_M.resize(4);
  // Q_M_dot.resize(4);
  // Q_M_2.resize(4);
  // Q_M_dot_2.resize(4);  
  // Q_M_A.resize(4, 4);
  // Q_M_B.resize(4, 1);
  // Q_M_C.resize(4);

  Cut_Off_Freq2 = Cut_Off_Freq * Cut_Off_Freq;

  angle_d << 0, 0;
  angle_d_lpf << 0, 0;

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



//////////////////////////////////////////
////////////////Admittance////////////////
//////////////////////////////////////////
	A_x <<                0, 									1,
		  - virtual_spring[0]/virtual_mass[0], - virtual_damper[0]/virtual_mass[0];

	B_x << 0, 1/virtual_mass[0];

	C_x << 1, 0;

	D_x << 0, 0;

//-----State Space Representation----//
	A_y <<               0, 									1,
		  - virtual_spring[1]/virtual_mass[1], - virtual_damper[1]/virtual_mass[1];

	B_y << 0, 1/virtual_mass[1];

	C_y << 1, 0;

	D_y << 0, 0;

	X_from_model_matrix << 0, 0;
	X_dot_from_model_matrix << 0, 0;

	Y_from_model_matrix << 0, 0;
	Y_dot_from_model_matrix << 0, 0;


//잘수정해보자//
  hysteresis_max << 0, 0;
  hysteresis_min << 0, 0;
    
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
  X_ref[0] = msg->position.at(0);
  X_ref[1] = msg->position.at(1);

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

  tau_measured[0] = msg->effort.at(0) * Kt;    //Kt = 0.0044 from 자시
  tau_measured[1] = msg->effort.at(0) * Kt;    //Kt = 0.0044 from 자시

  velocity_measured[0] = msg->velocity.at(0);
  // Jacobian 계산하기()
  J = Jacobian(angle_measured[0], angle_measured[1]);
  // Jacobian transpose()
  JT = J.transpose();
  JTI = J.inverse();
}





void TorqJ::Calc_Ext_Force()
{

  //---Dead Zone by hysteresis---//
  if (tau_measured[0] <= hysteresis_max[0] && tau_measured[0] >= hysteresis_min[0]) tau_measured[0] = 0;
  else if(tau_measured[0] > hysteresis_max[0]) tau_measured[0] -= hysteresis_max[0];
  else if(tau_measured[0] < hysteresis_min[0]) tau_measured[0] -= hysteresis_min[0];

  if (tau_measured[1] <= hysteresis_max[1] && tau_measured[1] >= hysteresis_min[1]) tau_measured[1] = 0;
  else if(tau_measured[1] > hysteresis_max[1]) tau_measured[1] -= hysteresis_max[1];
  else if(tau_measured[1] < hysteresis_min[1]) tau_measured[1] -= hysteresis_min[1];


  //---gravity model---//
   	 tau_gravity << (mass1 * CoM1 * cos(angle_measured[0]) + mass2 * Link1 * cos(angle_measured[0]) + mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1])) * 9.81 - offset_1,
 	 			          	mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1]) * 9.81 - offset_2;

  //---Calc Force_ext---//
  tau_ext = tau_measured - tau_gravity;

  Force_ext = JTI * tau_ext;

}

void TorqJ::Admittance_control()
{
	// X 방향 admittance model 적용--------------------------------
	X_dot_from_model_matrix = A_x * X_from_model_matrix + B_x * Force_ext[0];

	X_from_model_matrix = X_from_model_matrix + X_dot_from_model_matrix * time_loop;

	position_from_model[0] = X_from_model_matrix[0];

	X_cmd[0] = X_ref[0] - position_from_model[0];

	//----------------------------------------------------------------
	// Y 방향 admittance model 적용--------------------------------
	Y_dot_from_model_matrix = A_y * Y_from_model_matrix + B_y * Force_ext[1];

	Y_from_model_matrix = Y_from_model_matrix + Y_dot_from_model_matrix * time_loop;

	position_from_model[1] = Y_from_model_matrix[0];

	X_cmd[1] = X_ref[1] - position_from_model[1];
  
}

/////////////////////////////////////////////////////
//자신있다? 이 밑으로 X_ref 말고 X_cmd로 바꾸도록 하자//
/////////////////////////////////////////////////////
//1. hysteresis max min 파라미터 맞추기
//2. 외력추정 확인
//3. Admittance(토픽만) + virtual m d k 상수조절
//4. 어드미턴스 실시


void TorqJ::calc_des()
{


//-------커맨드 생성기------//
   i++;
   double Amp = 0.08;
   double period = 8;
   X_ref[0] = Amp * (sin(2* M_PI * 0.005 * i / period) + 1);
   X_ref[1] = 0.15;
//--------------------------------------

  JT.resize(2,2);


//-----Inverse Kinematics-----//  
  angle_ref[1] = acos((pow(X_ref[0],2) + pow(X_ref[1], 2) - pow(l1, 2)- pow(l2, 2)) / (2 * l1 * l2));
  angle_ref[0] = atan(X_ref[1] / X_ref[0]) - atan(l2* sin(angle_ref[1]) / (l1 + l2 * cos(angle_ref[1])));

  DoB();
}

void TorqJ::DoB()
{

  std::cout<<angle_d<<std::endl<<"angle_d"<<std::endl;
  std::cout<<angle_d_lpf<<std::endl<<"angle_d_lpf"<<std::endl;
  std::cout<<d_hat<<std::endl<<"d_hat"<<std::endl;

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
  //angle_d[0] = angle_ref[0] - d_hat[0]; // 14:17 d_hat이 nan
  angle_d[0] = angle_ref[0] - d_hat[0];
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

  FK_EE_pos = EE_pos(angle_measured[0], angle_measured[1]);

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
  joint_cmd.position.push_back(angle_d[0]); // 커맨드포지션
  joint_cmd.position.push_back(angle_d[1]); // 측정포지션
  joint_cmd.velocity.push_back(X_error_p[0]); // 포지션 에러
  joint_cmd.velocity.push_back(bw_2nd_output[2]); // 포지션 에러 필터값
  joint_cmd.effort.push_back(angle_d[0]); // joint space
  joint_cmd.effort.push_back(angle_ref[0]); // joint space

  joint_command_pub_.publish(joint_cmd);

  joint_measured.header.stamp = ros::Time::now();
  joint_measured.position.push_back(FK_EE_pos[0]); // 포지션 에러값
  joint_measured.position.push_back(X_ref[0]); // 포지션 에러 필터값 
  joint_measured.velocity.push_back(FK_EE_pos[1]); // 
  joint_measured.velocity.push_back(X_ref[1]);
  joint_measured_pub_.publish(joint_measured);
}


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ_icros");
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