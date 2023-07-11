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

#include "two_link/torque_jacobian.h"


double time_loop = 0;
double time_f = 0;
double time_i = 0;
double i = 0;
double j = 0;
int safety = 0;
int safety_force = 0;

TorqJ::TorqJ()
    : node_handle_(""),
      priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  // End Effector Position PID gain parameter
  position_p_gain = node_handle_.param<double>("position_p_gain", 1);
  position_i_gain = node_handle_.param<double>("position_i_gain", 1);
  position_d_gain = node_handle_.param<double>("position_d_gain", 1);

  


  // Q-filter
  Cut_Off_Freq = node_handle_.param<double>("Cut_Off_Freq", 1);

  //Model of DOB
  polar_moment_1 = node_handle_.param<double>("polar_moment_1", 1);
  polar_moment_2 = node_handle_.param<double>("polar_moment_2", 1);


  //admittance control parameter--//
  virtual_mass_x = node_handle_.param<double>("virtual_mass_x", 1);
  virtual_damper_x = node_handle_.param<double>("virtual_damper_x", 1);
  virtual_spring_x = node_handle_.param<double>("virtual_spring_x", 1);

  virtual_mass_y = node_handle_.param<double>("virtual_mass_y", 1);
  virtual_damper_y = node_handle_.param<double>("virtual_damper_y", 1);
  virtual_spring_y = node_handle_.param<double>("virtual_spring_y", 1);

  virtual_mass_z = node_handle_.param<double>("virtual_mass_z", 1);
  virtual_damper_z = node_handle_.param<double>("virtual_damper_z", 1);
  virtual_spring_z = node_handle_.param<double>("virtual_spring_z", 1);

  //---Current low pass filter--//
  cut_off_freq_current = node_handle_.param<double>("cut_off_freq_current", 1);


  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();


  virtual_mass << virtual_mass_x, virtual_mass_y, virtual_mass_z;
  virtual_damper << virtual_damper_x, virtual_damper_y, virtual_damper_z;
  virtual_spring << virtual_spring_x, virtual_spring_y, virtual_spring_z;


  ROS_INFO("TorqJ node start");
  ROS_INFO("===========================================");
  ROS_INFO("[3.2s] rising to init pose");
  ROS_INFO("[5.0s] DOB On");
  ROS_INFO("[10.0s] Force estimation start");
  ROS_INFO("===========================================");
  ROS_INFO("==============service command=============");
  ROS_INFO("movingService: Move sinusoidal");
  ROS_INFO("admitService: put x_m, x_d, x_k, y so on");
  ROS_INFO("compliant?: 0.5, 2, 0, Stiff?: 1000 1000 1000");
  ROS_INFO("------------");
  ROS_INFO("Before you use admitService, put E.E to reference position");


  //---------------------//
  //-----Matrix init-----//
  //---------------------//

  // angle_ref << 0, 0, 0, 0, 0, 0;
  // X_ref << 0.0, 0.0, 0.0;

  // Cut_Off_Freq2 = Cut_Off_Freq * Cut_Off_Freq;

  // angle_d << 0, 0, 0, 0, 0, 0;
  // angle_d_lpf << 0, 0;


  // //--For Model of DOB--//
  // Q_M << 0, 0, 0, 0;
  // Q_M_2 << 0, 0, 0, 0;

  // Q_M_dot << 0, 0, 0, 0;
  // Q_M_dot_2 << 0, 0, 0, 0;

  // Q_M_B << 0, 0, 0, 1;


  // Q_M_A << 0, 1, 0, 0,
  //     0, 0, 1, 0,
  //     0, 0, 0, 1,
  //     -position_i_gain * Cut_Off_Freq2 / position_d_gain,
  //     -(position_p_gain * Cut_Off_Freq2 + sqrt(2) * position_i_gain * Cut_Off_Freq) / position_d_gain,
  //     -(position_d_gain * Cut_Off_Freq2 + sqrt(2) * position_p_gain * Cut_Off_Freq + position_i_gain) / position_d_gain,
  //     -(position_p_gain + sqrt(2) * position_d_gain * Cut_Off_Freq) / position_d_gain;

  // Q_M_C << Cut_Off_Freq2 * position_i_gain / position_d_gain,
  //     Cut_Off_Freq2 * position_p_gain / position_d_gain,
  //     Cut_Off_Freq2,
  //     Cut_Off_Freq2 * polar_moment_1 / position_d_gain;


  // //---For Q-filter--//
  // Q_angle_d << 0, 0;
  // Q_angle_d_2 << 0, 0;
  // Q_angle_d_dot << 0, 0;
  // Q_angle_d_dot_2 << 0, 0;
  // Q_angle_d_A << 0, 1,
  //     -Cut_Off_Freq2, -sqrt(2) * Cut_Off_Freq;

  // Q_angle_d_B << 0, Cut_Off_Freq2;

  // Q_angle_d_C << 1, 0;

  // Q_angle_d_C = Q_angle_d_C.transpose();

  // //////////////////////////////////////////
  // ////////////////Admittance////////////////
  // //////////////////////////////////////////
  // A_x << 0, 1,
  //     -virtual_spring[0] / virtual_mass[0], -virtual_damper[0] / virtual_mass[0];

  // B_x << 0, 1 / virtual_mass[0];

  // C_x << 1, 0;

  // D_x << 0, 0;

  // A_y << 0, 1,
  //     -virtual_spring[1] / virtual_mass[1], -virtual_damper[1] / virtual_mass[1];

  // B_y << 0, 1 / virtual_mass[1];

  // C_y << 1, 0;

  // D_y << 0, 0;

  // A_z << 0, 1,
  //     -virtual_spring[2] / virtual_mass[2], -virtual_damper[2] / virtual_mass[2];

  // B_z << 0, 1 / virtual_mass[2];

  // C_z << 1, 0;

  // D_z << 0, 0;



  // X_from_model_matrix << 0, 0;
  // X_dot_from_model_matrix << 0, 0;

  // Y_from_model_matrix << 0, 0;
  // Y_dot_from_model_matrix << 0, 0;

  // Z_from_model_matrix << 0, 0;
  // Z_dot_from_model_matrix << 0, 0;


  // //------------------//
  // // Tuning parameter //
  // //------------------//

  // //--Dead Zone--//
  // hysteresis_max << 0.35, 0.27, 0.2, 0, 0, 0; //나바
  // hysteresis_min << -0.24, -0.35, -0.2, 0, 0, 0;

  // //--Angle saturation--//
  // angle_max << 1.6, 1.9, 0.75, 0, 0, 0; //나바
  // angle_min << -0.7, -1.8, -2, 0, 0, 0;

  // //--F_ext saturation--//
  // Force_max << 2.0, 1.0, 0; //나바
  // Force_min << -2.0, -1.0, 0;


  // theta_d = M_PI/2; //Command End Effector orientation

  // angle_ref << 0, 0, 0, 0, 0, 0;
  // X_test << 0, 0, 0, 0, 0, 0;
  // FK_EE_pos << 0, 0, 0;
  // FK_EE_ori << 0, 0, 0;
    ROS_INFO("Before you use admitService, put E.E to reference position");
  X_test.resize(6,1);
  angle_ref.resize(6,1);
  FK_EE_pos.resize(3,1);
  FK_EE_ori.resize(3,1);

  angle_measured.resize(6,1);
  tau_measured.resize(6,1);
  
  J.resize(6,6);
  JT.resize(6,6);
  JTI.resize(6,6);

  X_test << 0, 0, 0, 0, 0, 0;
  angle_ref << 0, 0, 0, 0, 0, 0;
  FK_EE_pos << 0, 0, 0;
  FK_EE_ori << 0, 0, 0;
      ROS_INFO("fffffff");

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
}

void TorqJ::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &TorqJ::jointCallback, this);

  movingService = node_handle_.advertiseService("/movingService", &TorqJ::movingServiceCallback, this);
  admitService = node_handle_.advertiseService("/admitService", &TorqJ::AdmittanceCallback, this);
}

void TorqJ::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  //Servo state subscriber
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);
  angle_measured[2] = msg->position.at(2);
  angle_measured[3] = msg->position.at(3);
  angle_measured[4] = msg->position.at(4);
  angle_measured[5] = msg->position.at(5);


  tau_measured[0] = msg->effort.at(0) * Kt_1; 
  tau_measured[1] = msg->effort.at(1) * Kt_2; 
  tau_measured[2] = msg->effort.at(2) * Kt_2; 
  tau_measured[3] = msg->effort.at(3) * Kt_1; 
  tau_measured[4] = msg->effort.at(4) * Kt_2; 
  tau_measured[5] = msg->effort.at(5) * Kt_2; 


}



  void TorqJ::second_order_butterworth()
{
  //////////////////////////////////////////
  ////////////////ButterWorth////////////////
  //////////////////////////////////////////
  wc = tan(M_PI * cut_off_freq_current * time_loop);
  wc2 = wc*wc;

  b0_2nd = (wc2) / (1 + sqrt(2)*wc + wc2);
  b1_2nd = 2 * b0_2nd;
  b2_2nd = b0_2nd;
  a0_2nd = 1;
  a1_2nd = 2 * (wc2 - 1) / (1 + sqrt(2) * wc + wc2);
  a2_2nd = (1 - sqrt(2) * wc + wc2) / (1 + sqrt(2) * wc + wc2);


  //--Joint 1--//
  bw2_filtered_current_1_input[2] = tau_measured[0];

  bw2_filtered_current_1_output[2] = b0_2nd * bw2_filtered_current_1_input[2] + b1_2nd * bw2_filtered_current_1_input[1] + b2_2nd * bw2_filtered_current_1_input[0] - a1_2nd * bw2_filtered_current_1_output[1] - a2_2nd * bw2_filtered_current_1_output[0];

  bw2_filtered_current_1_output[0] = bw2_filtered_current_1_output[1];
  bw2_filtered_current_1_output[1] = bw2_filtered_current_1_output[2];

  bw2_filtered_current_1_input[0] = bw2_filtered_current_1_input[1];
  bw2_filtered_current_1_input[1] = bw2_filtered_current_1_input[2];

  //--Joint 2--//
  bw2_filtered_current_2_input[2] = tau_measured[1];

  bw2_filtered_current_2_output[2] = b0_2nd * bw2_filtered_current_2_input[2] + b1_2nd * bw2_filtered_current_2_input[1] + b2_2nd * bw2_filtered_current_2_input[0] - a1_2nd * bw2_filtered_current_2_output[1] - a2_2nd * bw2_filtered_current_2_output[0];

  bw2_filtered_current_2_output[0] = bw2_filtered_current_2_output[1];
  bw2_filtered_current_2_output[1] = bw2_filtered_current_2_output[2];

  bw2_filtered_current_2_input[0] = bw2_filtered_current_2_input[1];
  bw2_filtered_current_2_input[1] = bw2_filtered_current_2_input[2];

  //--Joint 3--//
  bw2_filtered_current_3_input[2] = tau_measured[2];

  bw2_filtered_current_3_output[2] = b0_2nd * bw2_filtered_current_3_input[2] + b1_2nd * bw2_filtered_current_3_input[1] + b2_2nd * bw2_filtered_current_3_input[0] - a1_2nd * bw2_filtered_current_3_output[1] - a2_2nd * bw2_filtered_current_3_output[0];

  bw2_filtered_current_3_output[0] = bw2_filtered_current_3_output[1];
  bw2_filtered_current_3_output[1] = bw2_filtered_current_3_output[2];

  bw2_filtered_current_3_input[0] = bw2_filtered_current_3_input[1];
  bw2_filtered_current_3_input[1] = bw2_filtered_current_3_input[2];


  filtered_current << bw2_filtered_current_1_output[2], bw2_filtered_current_2_output[2], bw2_filtered_current_3_output[2], 0, 0, 0; //나바
 // std::cout<<filtered_current<<std::endl<<"butterworthbutterworth"<<std::endl;
}





void TorqJ::Calc_Ext_Force()
{

  J = Jacobian(angle_measured[0], angle_measured[1], angle_measured[2], angle_measured[3], angle_measured[4], angle_measured[5]); //나바
  JT = J.transpose();
  JTI = JT.completeOrthogonalDecomposition().pseudoInverse();




  //---gravity model---//
  tau_gravity << (mass1 * CoM1 * cos(angle_measured[0]) + mass2 * Link1 * cos(angle_measured[0]) + mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1]) + delta) * 9.81 - offset_1,
      mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1] + delta) * 9.81 - offset_2,
      -offset_3, 0, 0, 0; //나바

  //---Calc Force_ext---//
  tau_ext = tau_measured - tau_gravity;

  tau_ext_not_deadzone = tau_ext; // To Compare w/ & w/o dead zone

  //---Dead Zone by hysteresis and others---//
  if (tau_ext[0] <= hysteresis_max[0] && tau_ext[0] >= hysteresis_min[0]) tau_ext[0] = 0;
  else if (tau_ext[0] > hysteresis_max[0]) tau_ext[0] -= hysteresis_max[0];
  else if (tau_ext[0] < hysteresis_min[0]) tau_ext[0] -= hysteresis_min[0];

  if (tau_ext[1] <= hysteresis_max[1] && tau_ext[1] >= hysteresis_min[1]) tau_ext[1] = 0;
  else if (tau_ext[1] > hysteresis_max[1]) tau_ext[1] -= hysteresis_max[1];
  else if (tau_ext[1] < hysteresis_min[1]) tau_ext[1] -= hysteresis_min[1];

  if (tau_ext[2] <= hysteresis_max[2] && tau_ext[2] >= hysteresis_min[2]) tau_ext[2] = 0;
  else if (tau_ext[2] > hysteresis_max[2]) tau_ext[2] -= hysteresis_max[2];
  else if (tau_ext[2] < hysteresis_min[2]) tau_ext[2] -= hysteresis_min[2];

  tau_ext << 0, 0, 0, 0, 0, 0; //나바

  Force_ext_not_deadzone = JTI * tau_ext_not_deadzone;

  if(safety > 2000) Force_ext = JTI * tau_ext;
  else Force_ext << 0, 0, 0, 0, 0, 0;

  //--Force saturation--//
  if (Force_ext[0] > Force_max[0]) Force_ext[0] = Force_max[0];
  else if (Force_ext[0] < Force_min[0]) Force_ext[0] = Force_min[0];

  if (Force_ext[1] > Force_max[1]) Force_ext[1] = Force_max[1];
  else if (Force_ext[1] < Force_min[1]) Force_ext[1] = Force_min[1];

  Force_ext[2] = 0; //나바


}

void TorqJ::Admittance_control()
{
  // X axis admittance model --------------------------------
  X_dot_from_model_matrix = A_x * X_from_model_matrix + B_x * Force_ext[0];

  X_from_model_matrix = X_from_model_matrix + X_dot_from_model_matrix * time_loop;

  position_from_model[0] = X_from_model_matrix[0];

  X_cmd[0] = X_ref[0] - position_from_model[0];

  //----------------------------------------------------------------
  // Y axis admittance model --------------------------------
  Y_dot_from_model_matrix = A_y * Y_from_model_matrix + B_y * Force_ext[1];

  Y_from_model_matrix = Y_from_model_matrix + Y_dot_from_model_matrix * time_loop;

  position_from_model[1] = Y_from_model_matrix[0];

  X_cmd[1] = X_ref[1] - position_from_model[1];

  //----------------------------------------------------------------
  // Z axis admittance model --------------------------------

// 나바
  position_from_model[2] = 0;

  X_cmd[2] = 0;


}



void TorqJ::calc_des()
{

  //-----End Effector command generator-----//
   
if(i < 1040)
{
  i++;
   X_ref[0] = 0.114329;
   X_ref[1] = 0.218 + i / 20000;  //init: 0.218m, goal: 0.27m
}


if(movingFlag && safety > 1000) 
{
  //--If rosservice "movingService" calls, Path input is given sinusoidal in the x direction
  //--For safe, it works after 5s from start.
  j++;

   double Amp = 0.07;
   double period = 5;
   X_ref[0] = Amp * (sin(2* M_PI * 0.005 * j / period + M_PI/2)) + 0.114329 - Amp;
   X_ref[1] = 0.27;

}
// --initial position--//
// [ INFO] [1687091726.145587992]: -0.217825, 1.685845, 0.111981
// [ WARN] [1687091726.145748576]: FK: 0.114329, 0.217195




//     //  Command Safety Function
//     if (std::isnan(X_Command[0]) || std::isnan(X_Command[1])) ROS_ERROR("Inverse Kinematics Error(NaN)");
//     else X_Command = X_cmd; // If wanna admittance, put "X_cmd". If don't wanna admittance, put "X_ref"



//   //-----Inverse Kinematics-----//
// POSITION_2[0] = X_Command[0] - Link3 * cos(theta_d);
// POSITION_2[1] = X_Command[1] - Link3 * sin(theta_d);

// r2 = pow(POSITION_2[0], 2) + pow(POSITION_2[1], 2);
// D = (r2 - Link1 * Link1 - Link2 * Link2) / (2 * Link1 * Link2);

// angle_ref[1] = atan2(sqrt(1 - D*D), D);
// angle_ref[0] = atan2(POSITION_2[1], POSITION_2[0]) - atan2(Link2 * sin(angle_ref[1]), Link1 + Link2 * cos(angle_ref[1]));
// angle_ref[2] = theta_d - (angle_ref[0] + angle_ref[1]);



  angle_safe_func();      
  DoB();
  Calc_Ext_Force();
  second_order_butterworth();
}

void TorqJ::DoB()
{
  
  // 1st motor
  // State space (Gn)//
  Q_M_dot = Q_M_A * Q_M + Q_M_B * angle_measured[0];
  Q_M += Q_M_dot * time_loop;
  angle_d_hat[0] = Q_M_C.dot(Q_M); 

  // State space(theta_d) //
  Q_angle_d_dot = Q_angle_d_A * Q_angle_d + Q_angle_d_B * angle_d[0];
  Q_angle_d += Q_angle_d_dot * time_loop;
  angle_d_lpf[0] = Q_angle_d_C.dot(Q_angle_d);

  d_hat[0] = angle_d_hat[0] - angle_d_lpf[0]; // d_hat: estimated dist

  angle_d[0] = angle_ref[0] - d_hat[0];


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 2nd motor
  // State space (Gn)//
  Q_M_dot_2 = Q_M_A * Q_M_2 + Q_M_B * angle_measured[1];
  Q_M_2 += Q_M_dot_2 * time_loop;
  angle_d_hat[1] = Q_M_C.dot(Q_M_2);

  // State space(theta_d) //
  Q_angle_d_dot_2 = Q_angle_d_A * Q_angle_d_2 + Q_angle_d_B * angle_d[1];
  Q_angle_d_2 += Q_angle_d_dot_2 * time_loop;
  angle_d_lpf[1] = Q_angle_d_C.dot(Q_angle_d_2);

  d_hat[1] = angle_d_hat[1] - angle_d_lpf[1]; // d_hat: estimated dist

  angle_d[1] = angle_ref[1] - d_hat[1];


//나바
  angle_d[2] = angle_ref[2];

  angle_d[3] = angle_ref[3];

  angle_d[4] = angle_ref[4];

  angle_d[5] = angle_ref[5];

}

void TorqJ::angle_safe_func()
{
    if (safety < 1000) 
    {
      angle_command = angle_ref;
    
      Q_M << 0, 0, 0, 0;
      Q_M_2 << 0, 0, 0, 0;
      Q_angle_d << 0, 0;
      Q_angle_d_2 << 0, 0;
      
    }
    else angle_command = angle_ref;
    // DoB safety lock
    // An impulse input is applied at startup.


  if (safety < 1300)  // Do not work safety function until 6.5s from startup.
  {
      angle_safe[0] = angle_command[0];
      angle_safe[1] = angle_command[1];
      angle_safe[2] = angle_command[2];
      angle_safe[3] = angle_command[3];
      angle_safe[4] = angle_command[4];
      angle_safe[5] = angle_command[5];
  }
  else
  {
    //--For warning message--//
    if (std::isnan(angle_command[0])) ROS_ERROR("angle 1 is NaN!!");
    else if (angle_command[0] > angle_max[0] || angle_command[0] < angle_min[0]) ROS_ERROR("angle 1 LIMIT");

    if (std::isnan(angle_command[1])) ROS_ERROR("angle 2 is NaN!!");
    else if (angle_command[1] > angle_max[1] || angle_command[1] < angle_min[1]) ROS_ERROR("angle 2 LIMIT");

    if (std::isnan(angle_command[2])) ROS_ERROR("angle 3 is NaN!!");
    else if (angle_command[2] > angle_max[2] || angle_command[2] < angle_min[2]) ROS_ERROR("angle 3 LIMIT");

    if (std::isnan(angle_command[3])) ROS_ERROR("angle 4 is NaN!!");
    else if (angle_command[3] > angle_max[3] || angle_command[3] < angle_min[3]) ROS_ERROR("angle 4 LIMIT");

    if (std::isnan(angle_command[4])) ROS_ERROR("angle 5 is NaN!!");
    else if (angle_command[4] > angle_max[4] || angle_command[4] < angle_min[4]) ROS_ERROR("angle 5 LIMIT");

    if (std::isnan(angle_command[5])) ROS_ERROR("angle 6 is NaN!!");
    else if (angle_command[5] > angle_max[5] || angle_command[5] < angle_min[5]) ROS_ERROR("angle 6 LIMIT");


    //--for constraint--//
    if (
        (std::isnan(angle_command[0]) && std::isnan(angle_command[1]) && std::isnan(angle_command[2])) ||
        (angle_command[0] > angle_max[0] || angle_command[0] < angle_min[0]) ||
        (angle_command[1] > angle_max[1] || angle_command[1] < angle_min[1]) ||
        (angle_command[2] > angle_max[2] || angle_command[2] < angle_min[2])
        )
    {
      ROS_INFO("IK error");
    }
    else
    {
      angle_safe[0] = angle_command[0];
      angle_safe[1] = angle_command[1];
      angle_safe[2] = angle_command[2];
      angle_safe[3] = angle_command[3];
      angle_safe[4] = angle_command[4];
      angle_safe[5] = angle_command[5];
    }
  }
}





 bool TorqJ::movingServiceCallback(two_link::movingFlag::Request  &req,
          two_link::movingFlag::Response &res)
 {
  if(movingFlag)
  {
    movingFlag = false;
    ROS_INFO("movingFlag false");
  }
  else
  {
    movingFlag = true;
    ROS_INFO("movingFlag true");
  }
	return true;
 }


 bool TorqJ::AdmittanceCallback(two_link::admittanceTest::Request  &req,
          two_link::admittanceTest::Response &res)
 {

  virtual_mass[0] = req.x_m;
  virtual_damper[0] = req.x_d;
  virtual_spring[0] = req.x_k;
  
  virtual_mass[1] = req.y_m;
  virtual_damper[1] = req.y_d;
  virtual_spring[1] = req.y_k;

  ROS_INFO("Admittance Parameter updated!");
  ROS_INFO("x: %lf, %lf, %lf \n y: %lf, %lf, %lf", virtual_mass[0], virtual_damper[0], virtual_spring[0], virtual_mass[1], virtual_damper[1], virtual_spring[1]);

  //////////////////////////////////////////
  ////////////////Admittance////////////////
  //////////////////////////////////////////
  ///////Init Again Admittance matrix///////
  A_x << 0, 1,
      -virtual_spring[0] / virtual_mass[0], -virtual_damper[0] / virtual_mass[0];

  B_x << 0, 1 / virtual_mass[0];

  C_x << 1, 0;

  D_x << 0, 0;

  //-----State Space Representation----//
  A_y << 0, 1,
      -virtual_spring[1] / virtual_mass[1], -virtual_damper[1] / virtual_mass[1];

  B_y << 0, 1 / virtual_mass[1];

  C_y << 1, 0;

  D_y << 0, 0;

//  X_from_model_matrix << 0, 0;
//  X_dot_from_model_matrix << 0, 0;

//  Y_from_model_matrix << 0, 0;
//  Y_dot_from_model_matrix << 0, 0;

	return true;
 }

double t = 0;
void TorqJ::CommandGenerator()
{
  
  t++;
  double Amp = 0.07;
  double period = 5;
  X_test[0] = 0; //Amp * (sin(2* M_PI * 0.005 * t / period + M_PI/2)) + 0.114329 - Amp;

  double Amp1 = 0.07;
  double period1 = 5;
  X_test[1] = 0.15; //Amp1 * (sin(2* M_PI * 0.005 * t / period1 + M_PI/2)) + 0.114329 - Amp1;

  double Amp2 = 0.05;
  double period2 = 5;
  X_test[2] = Amp2 * (sin(2* M_PI * 0.005 * t / period2 + M_PI/2)) + 0.114329 - Amp2 + 0.1;

  X_test[3] = 0 * (t / 0.005) / (180 * M_PI);
  X_test[4] = 0 * (t / 0.005) / (180 * M_PI);
  X_test[5] = 0 * (t / 0.005) / (180 * M_PI); //1초에 몇 도(degree) 씩 회전시킬지
}


void TorqJ::PublishCmdNMeasured()
{

  sensor_msgs::JointState joint_cmd;
  sensor_msgs::JointState joint_measured;

  geometry_msgs::Twist first_publisher;
  geometry_msgs::Twist second_publisher;
  // tau_des를 publish
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.position.push_back(angle_ref[0]); 
  joint_cmd.position.push_back(angle_ref[1]); 
  joint_cmd.position.push_back(angle_ref[2]); 
  joint_cmd.position.push_back(angle_ref[3]); 
  joint_cmd.position.push_back(angle_ref[4]); 
  joint_cmd.position.push_back(angle_ref[5]); 

  joint_cmd.velocity.push_back(X_test[0] - FK_EE_pos[0]); 
  joint_cmd.velocity.push_back(X_test[1] - FK_EE_pos[1]); 
  joint_cmd.velocity.push_back(X_test[2] - FK_EE_pos[2]); 
  joint_cmd.velocity.push_back(X_test[3] - FK_EE_ori[0]); 
  joint_cmd.velocity.push_back(X_test[4] - FK_EE_ori[1]); 
  joint_cmd.velocity.push_back(X_test[5] - FK_EE_ori[2]); 


  joint_cmd.effort.push_back(FK_EE_pos[0]); 
  joint_cmd.effort.push_back(FK_EE_pos[1]); 
  joint_cmd.effort.push_back(FK_EE_pos[2]); 
  joint_cmd.effort.push_back(FK_EE_ori[0]); 
  joint_cmd.effort.push_back(FK_EE_ori[1]); 
  joint_cmd.effort.push_back(FK_EE_ori[2]); 

  joint_command_pub_.publish(joint_cmd);


}

void TorqJ::solveInverseKinematics()
{
  // EE_position과 orientation이 들어왔을 것: X_ref, Orientation_ref

  angle_ref = InverseKinematics(X_test[0], X_test[1], X_test[2],
                                X_test[3], X_test[4], X_test[5]);

  ROS_INFO("=============angle command from inverse kinematics=========");
  ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", angle_ref[0], angle_ref[1], angle_ref[2], angle_ref[3], angle_ref[4], angle_ref[5]);

  FK_EE_pos = EE_pos(angle_ref[0], angle_ref[1], angle_ref[2], angle_ref[3], angle_ref[4], angle_ref[5]);
  FK_EE_ori = EE_orientation(angle_ref[0], angle_ref[1], angle_ref[2], angle_ref[3], angle_ref[4], angle_ref[5]);

  ROS_INFO("============Command position - FK position ================");
  ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", X_test[0] - FK_EE_pos[0], X_test[1] - FK_EE_pos[1], X_test[2] - FK_EE_pos[2], X_test[3] - FK_EE_ori[0], X_test[4] - FK_EE_ori[1], X_test[5] - FK_EE_ori[2]);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(200);

  time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
    safety++;

    //---Measure sampling time---//
    time_f = ros::Time::now().toSec();
    time_loop = time_f - time_i;
    time_i = ros::Time::now().toSec();

    // torqJ.Admittance_control();
    // torqJ.calc_des();

    torqJ.CommandGenerator();
    torqJ.solveInverseKinematics();   
    torqJ.PublishCmdNMeasured();
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}