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

#include "dasom_controllers/torque_jacobian.h"

double time_loop = 0;
double time_f = 0;
double time_i = 0;
double i = 0;
double j = 0;
int safety = 0;
int safety_force = 0;
int checkFirstPoseFlag;

TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  // End Effector Position PID gain parameter
  position_p_gain = node_handle_.param<double>("position_p_gain", 1);
  position_i_gain = node_handle_.param<double>("position_i_gain", 1);
  position_d_gain = node_handle_.param<double>("position_d_gain", 1);

  position_p_gain_2nd = node_handle_.param<double>("position_p_gain_2nd", 1);
  position_i_gain_2nd = node_handle_.param<double>("position_i_gain_2nd", 1);
  position_d_gain_2nd = node_handle_.param<double>("position_d_gain_2nd", 1);

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

  ds_wb_ = new dasom::DasomWorkbench;
  ds_jnt2_ = new DasomJoint(8,8);

  // ds_wb_->run();

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

  X_test.resize(6,1);
  X_cmd.resize(6,1);
  X_ref.resize(6,1);
  angle_ref.resize(6);
  angle_ref_i.resize(6,1);
  angle_d.resize(6,1);
  FK_pose.resize(6,1);
  haptic_command.resize(6);
  haptic_initPose.resize(6,1);
  initPose.resize(6, 1);
  EE_command_vel_limit.resize(6);

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
  EE_command.resize(6);
  gimbal_tf.resize(7);
  global_EE_tf.resize(7);
  paletrone_tf.resize(6);
  gimbal_EE_cmd.resize(6);
  d_hat.resize(2);
  angle_init.resize(6);
  angle_command.resize(6);


  angle_ref << 0, 0, 0, 0, 0, 0;
  angle_ref_i << 0, 0, 0, 0, 0, 0;

  Cut_Off_Freq2 = Cut_Off_Freq * Cut_Off_Freq;

  angle_d << 0, 0, 0, 0, 0, 0;
  angle_d_lpf << 0, 0;

  angle_safe << 0, 0, 0, 0, 0, 0;

  X_test << 0, 0, 0, 0, 0, 0;
  FK_pose << 0, 0.23, 0.30, M_PI/2, 0, 0;
  //--For Model of DOB--//
  Q_M << 0, 0, 0, 0;
  Q_M_2 << 0, 0, 0, 0;

  Q_M_dot << 0, 0, 0, 0;
  Q_M_dot_2 << 0, 0, 0, 0;

  Q_M_B << 0, 0, 0, 1;


  Q_M_A << 0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
      -position_i_gain_2nd * Cut_Off_Freq2 / position_d_gain_2nd,
      -(position_p_gain_2nd * Cut_Off_Freq2 + sqrt(2) * position_i_gain_2nd * Cut_Off_Freq) / position_d_gain_2nd,
      -(position_d_gain_2nd * Cut_Off_Freq2 + sqrt(2) * position_p_gain_2nd * Cut_Off_Freq + position_i_gain_2nd) / position_d_gain_2nd,
      -(position_p_gain_2nd + sqrt(2) * position_d_gain_2nd * Cut_Off_Freq) / position_d_gain_2nd;

  Q_M_C << Cut_Off_Freq2 * position_i_gain_2nd / position_d_gain_2nd,
      Cut_Off_Freq2 * position_p_gain_2nd / position_d_gain_2nd,
      Cut_Off_Freq2,
      Cut_Off_Freq2 * polar_moment_1 / position_d_gain_2nd;


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

  grey_button = true;
  white_button = true;
  initPoseFlag = false;



  //------------------//
  // Tuning parameter //
  //------------------//

  //--Dead Zone--//
  hysteresis_max << 0.35, 0.27, 0.2, 0, 0, 0; //나바
  hysteresis_min << -0.24, -0.35, -0.2, 0, 0, 0;

  //--Angle saturation--//
  // angle_max << M_PI/2, 2, -0.35, M_PI / 3, 0.5, 1.8; //나바
  // angle_min << -M_PI/2, 0.5, -2, -M_PI / 3, -0.5, 0.9;
  angle_max << 10, 100, 10, 10, 10, 10; //나바
  angle_min << -10, -100, -10, -10, -10, -10;

  //--F_ext saturation--//
  Force_max << 2.0, 1.0, 0; //나바
  Force_min << -2.0, -1.0, 0;


  initPose << 0, 0.23, 0.30, M_PI/2, 0, 0;
  EE_command << 0, 0.23, 0.30, M_PI/2, 0, 0; // ADD
  EE_command_vel_limit << 0, 0.23, 0.30, M_PI/2, 0, 0; // ADD
  // ROS_ERROR("EE_command: %lf %lf %lf %lf %lf %lf ", EE_command[0], EE_command[1], EE_command[2], EE_command[3], EE_command[4], EE_command[5]);


  angle_init = InverseKinematics(initPose);


  // -- 초기값 퍽 튀기 방지용 -- //
  // geometry_msgs::PoseStamped gimbal_tf_msg;
  // gimbal_pub.publish(gimbal_tf_msg);
  // 하지만 소용이 없었따..
  ros::Rate init_rate(1);

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
  dasom_EE_pos_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/dasom/EE_pose", 10); //use
  gimbal_pub = node_handle_.advertise<geometry_msgs::PoseStamped>("/dasom/global_gimbal_pose", 10); //use
  EE_command_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/dasom/EE_command", 10); //use


  test_Pub = node_handle_.advertise<geometry_msgs::Twist>("/dasom/test_Pub", 10); //use
  test_Pub2 = node_handle_.advertise<geometry_msgs::Twist>("/dasom/test_Pub2", 10); //use
}

void TorqJ::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &TorqJ::jointCallback, this, ros::TransportHints().tcpNoDelay());
  joystick_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 10, &TorqJ::joystickCallback, this, ros::TransportHints().tcpNoDelay());
  button_sub_ = node_handle_.subscribe("/phantom/button", 10, &TorqJ::buttonCallback, this, ros::TransportHints().tcpNoDelay());
  gimbal_sub_ = node_handle_.subscribe("/dasom/global_EE_frame/world", 10, &TorqJ::gimbalCallback, this, ros::TransportHints().tcpNoDelay()); //use
  paletrone_sub_ = node_handle_.subscribe("/dasombasePlate/world", 10, &TorqJ::paletroneCallback, this, ros::TransportHints().tcpNoDelay()); //use
  gimbal_cmd_sub_ = node_handle_.subscribe("/dasom/gimbal_EE_cmd", 10, &TorqJ::gimbal_cmdCallback, this, ros::TransportHints().tcpNoDelay()); //use


  movingService = node_handle_.advertiseService("/movingService", &TorqJ::movingServiceCallback, this);
  admitService = node_handle_.advertiseService("/admitService", &TorqJ::AdmittanceCallback, this);

}

void TorqJ::paletroneCallback(const geometry_msgs::PoseStamped &msg)
{
  // paletrone_tf[0] = msg.pose.position.x;
  // paletrone_tf[1] = msg.pose.position.y;
  // paletrone_tf[2] = msg.pose.position.z;

  // tf::Quaternion quat;
  // tf::quaternionMsgToTF(msg.pose.orientation,quat);

  // tf::Matrix3x3(quat).getRPY(paletrone_tf[3], paletrone_tf[4], paletrone_tf[5]);
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


  tau_measured[0] = msg->effort.at(0); 
  tau_measured[1] = msg->effort.at(1); 
  tau_measured[2] = msg->effort.at(2); 
  tau_measured[3] = msg->effort.at(3); 
  tau_measured[4] = msg->effort.at(4); 
  tau_measured[5] = msg->effort.at(5);
}

void TorqJ::joystickCallback(const geometry_msgs::Twist &msg)
{
  haptic_command[0] = msg.linear.x;
  haptic_command[1] = msg.linear.y;
  haptic_command[2] = msg.linear.z;
  // ROS_WARN("JOYhapticmd = %lf, %lf, %lf", haptic_command[0], haptic_command[1], haptic_command[2]);
  // ROS_WARN("JOYinitpose = %lf, %lf, %lf, %lf, %lf, %lf", initPose[0], initPose[1], initPose[2], initPose[3], initPose[4], initPose[5]);
}


void TorqJ::gimbalCallback(const geometry_msgs::PoseStamped &msg)
// global_EE_pose 받아오기
{

  global_EE_tf[0] = msg.pose.position.x;
  global_EE_tf[1] = msg.pose.position.y;
  global_EE_tf[2] = msg.pose.position.z;

  global_EE_tf[3] = msg.pose.orientation.x;
  global_EE_tf[4] = msg.pose.orientation.y;
  global_EE_tf[5] = msg.pose.orientation.z;
  global_EE_tf[6] = msg.pose.orientation.w;

}


void TorqJ::gimbal_cmdCallback(const geometry_msgs::PoseStamped &msg)
{

  gimbal_EE_cmd[0] = msg.pose.position.x;
  gimbal_EE_cmd[1] = msg.pose.position.y;
  gimbal_EE_cmd[2] = msg.pose.position.z;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation,quat);

  tf::Matrix3x3(quat).getRPY(gimbal_EE_cmd[3], gimbal_EE_cmd[4], gimbal_EE_cmd[5]);

}


void TorqJ::buttonCallback(const omni_msgs::OmniButtonEvent &msg)
{
  if(msg.grey_button == 1) 
  {
    if(grey_button)
    {
      grey_button = false;
      ROS_INFO("Grey: false");

      gimbal_tf = global_EE_tf;

      geometry_msgs::PoseStamped gimbal_tf_msg;


      gimbal_tf_msg.pose.position.x = gimbal_tf[0];
      gimbal_tf_msg.pose.position.y = gimbal_tf[1];
      gimbal_tf_msg.pose.position.z = gimbal_tf[2];

      gimbal_tf_msg.pose.orientation.x = gimbal_tf[3];
      gimbal_tf_msg.pose.orientation.y = gimbal_tf[4];
      gimbal_tf_msg.pose.orientation.z = gimbal_tf[5];
      gimbal_tf_msg.pose.orientation.w = gimbal_tf[6];

      gimbal_pub.publish(gimbal_tf_msg);
    //3번 내용.
    //broadcaster의 gimbalTF_Callback에서 정의중 
      // ROS_INFO("GIMBAL_TF = %lf, %lf, %lf, %lf, %lf, %lf", gimbal_tf[0], gimbal_tf[1], gimbal_tf[2], gimbal_tf[3], gimbal_tf[4], gimbal_tf[5]);
    }
    else
    {
      grey_button = true;
      ROS_INFO("Grey: True");
    }
  }

  if(msg.white_button == 1) 
  {
    if(white_button)
    {
      white_button = false;
      ROS_INFO("white: false");
      gripper_cmd = 2.3;
    }
    else
    {
      white_button = true;
      ROS_INFO("white: true");
      gripper_cmd = 0;
    }
  }

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

  J = Jacobian(angle_measured); //나바
  JT = J.transpose();
  JTI = JT.inverse(); // completeOrthogonalDecomposition().pseudoInverse();
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
  else if (tau_ext[2] > hysteresis_max[2]) tau_ext[2] -= hysteresis_min[2];

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

void TorqJ::DoB()
{
  // angle_d[0] = ds_jnt1_->updateDOB(time_loop, angle_ref[0], angle_measured[0]);
  //angle_d[0] = angle_ref[0];

  //d_hat[1] = ds_jnt2_->updateDOB(time_loop, angle_ref[1], angle_measured[1]);
  angle_ref[1] = angle_ref[1];//- d_hat[1];

  // angle_d[2] = ds_jnt3_->updateDOB(time_loop, angle_ref[2], angle_measured[2]);
  //angle_d[2] = angle_ref[2];

  // angle_d[3] = ds_jnt4_->updateDOB(time_loop, angle_ref[3], angle_measured[3]);
  //angle_d[3] = angle_ref[3];

  // angle_d[4] = ds_jnt5_->updateDOB(time_loop, angle_ref[4], angle_measured[4]);
  //angle_d[4] = angle_ref[4];

  // angle_d[5] = ds_jnt6_->updateDOB(time_loop, angle_ref[5], angle_measured[5]);
  //angle_d[5] = angle_ref[5];

}

void TorqJ::angle_safe_func()
{
    if (angle_ref[0] > angle_max[0] || angle_ref[0] < angle_min[0]) ROS_ERROR("angle 1 LIMIT");

    if (angle_ref[1] > angle_max[1] || angle_ref[1] < angle_min[1]) ROS_ERROR("angle 2 LIMIT");

    if (angle_ref[2] > angle_max[2] || angle_ref[2] < angle_min[2]) ROS_ERROR("angle 3 LIMIT");

    if (angle_ref[3] > angle_max[3] || angle_ref[3] < angle_min[3]) ROS_ERROR("angle 4 LIMIT");

    if (angle_ref[4] > angle_max[4] || angle_ref[4] < angle_min[4]) ROS_ERROR("angle 5 LIMIT");

    if (angle_ref[5] > angle_max[5] || angle_ref[5] < angle_min[5]) ROS_ERROR("angle 6 LIMIT");

    if (std::isnan(angle_ref[0]) || std::isnan(angle_ref[1]) || std::isnan(angle_ref[2]) ||
        std::isnan(angle_ref[3]) || std::isnan(angle_ref[4]) || std::isnan(angle_ref[5]))
      ROS_WARN("Out of Workspace");


    //for constraint--//
    if (
        (angle_ref[0] > angle_max[0] || angle_ref[0] < angle_min[0]) ||
        (angle_ref[1] > angle_max[1] || angle_ref[1] < angle_min[1]) ||
        (angle_ref[2] > angle_max[2] || angle_ref[2] < angle_min[2]) ||
        (angle_ref[3] > angle_max[3] || angle_ref[3] < angle_min[3]) ||
        (angle_ref[4] > angle_max[4] || angle_ref[4] < angle_min[4]) ||
        (angle_ref[5] > angle_max[5] || angle_ref[5] < angle_min[5]) ||
        std::isnan(angle_ref[0]) || std::isnan(angle_ref[1]) || std::isnan(angle_ref[2]) ||
        std::isnan(angle_ref[3]) || std::isnan(angle_ref[4]) || std::isnan(angle_ref[5])
        )
    {
      ROS_INFO("IK error");
    }
    else
    {
      angle_safe[0] = angle_ref[0];
      angle_safe[1] = angle_ref[1];
      angle_safe[2] = angle_ref[2];
      angle_safe[3] = angle_ref[3];
      angle_safe[4] = angle_ref[4];
      angle_safe[5] = angle_ref[5];
    }

}

bool TorqJ::movingServiceCallback(dasom_controllers::movingFlag::Request  &req,
                                  dasom_controllers::movingFlag::Response &res)
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


bool TorqJ::AdmittanceCallback(dasom_controllers::admittanceTest::Request  &req,
                                dasom_controllers::admittanceTest::Response &res)
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
  if(grey_button == true) 
  {
    EE_command = haptic_command + initPose; // EE_command: 햅틱커맨드가 아니고 실제로 ik 풀 때 들어가는 command
    ROS_INFO("Command Mode");
  }
  else 
  {
    EE_command << 0.1, 0.15, 0.20, M_PI / 2, 0, 0;//EE_command = gimbal_EE_cmd;
    ROS_INFO("Gimballing Mode");
  }

geometry_msgs::Twist EE_command_msg;

EE_command_msg.linear.x = EE_command[0];
EE_command_msg.linear.y = EE_command[1];
EE_command_msg.linear.z = EE_command[2];
EE_command_msg.angular.x = EE_command[3];
EE_command_msg.angular.y = EE_command[4];
EE_command_msg.angular.z = EE_command[5];
EE_command_pub_.publish(EE_command_msg);
}

bool start_plag = false;
void TorqJ::CommandVelocityLimit()
{
  // for (int i = 0; i < 3; i++)
  // {
  //   if((EE_command[i] - EE_command_vel_limit[i]) > 0.001) 
  //   {
  //     EE_command_vel_limit[i] = EE_command_vel_limit[i] + 0.0005;
  //   }
  //   else if ((EE_command[i] - EE_command_vel_limit[i]) < - 0.001) 
  //   {
  //     EE_command_vel_limit[i] = EE_command_vel_limit[i] - 0.0005; // 0.01 * time_loop              
  //   }
  //   else
  //   {
  //     EE_command_vel_limit[i] = EE_command[i];
  //   }
  // }

  EE_command_vel_limit[0] = EE_command[0];
  EE_command_vel_limit[1] = EE_command[1];
  EE_command_vel_limit[2] = EE_command[2];
  EE_command_vel_limit[3] = EE_command[3];
  EE_command_vel_limit[4] = EE_command[4];
  EE_command_vel_limit[5] = EE_command[5];

  // ROS_INFO("VLfinal_cmd =  %lf %lf %lf %lf %lf %lf", EE_command_vel_limit[0], EE_command_vel_limit[1], EE_command_vel_limit[2], EE_command_vel_limit[3], EE_command_vel_limit[4], EE_command_vel_limit[5]);
}

void TorqJ::solveInverseKinematics()
{
  geometry_msgs::Twist msg;

  // EE_position과 orientation이 들어왔을 것: X_ref, Orientation_ref

  // ROS_ERROR("From IK EE_command: %lf %lf %lf %lf %lf %lf ", EE_command[0], EE_command[1], EE_command[2], EE_command[3], EE_command[4], EE_command[5]);

  // ROS_WARN("IKfinal_cmd =  %lf %lf %lf %lf %lf %lf", EE_command_vel_limit[0], EE_command_vel_limit[1], EE_command_vel_limit[2], EE_command_vel_limit[3], EE_command_vel_limit[4], EE_command_vel_limit[5]);

  angle_ref = InverseKinematics(EE_command_vel_limit);
  // ROS_WARN("CGfinal_cmd =  %lf %lf %lf %lf %lf %lf", angle_ref[0], angle_ref[1], angle_ref[2], angle_ref[3], angle_ref[4], angle_ref[5]);

  FK_pose = EE_pose(angle_measured);

  msg.linear.x = FK_pose[0];
  msg.linear.y = FK_pose[1];
  msg.linear.z = FK_pose[2];
  msg.angular.x = FK_pose[3];
  msg.angular.y = FK_pose[4];
  msg.angular.z = FK_pose[5];

  dasom_EE_pos_pub_.publish(msg);

}

void TorqJ::initPoseFunc()
{
  // ROS_ERROR("initpose = %lf, %lf, %lf, %lf, %lf, %lf", initPose[0], initPose[1], initPose[2], initPose[3], initPose[4], initPose[5]);

  for (int i = 0; i < 6; i++)
  {
    if((angle_init[i] - angle_ref[i]) > 0.001) 
    {
      angle_ref[i] = angle_ref[i] + 0.0005;
    }
    else if ((angle_init[i] - angle_ref[i]) < - 0.001) 
    {
      angle_ref[i] = angle_ref[i] - 0.0005;   //0.01 * time_loop              
    }
    else
    {
      angle_ref[i] = angle_init[i];
    }
  }

  if (
      (abs(angle_init[0] - angle_ref[0]) < 0.001) &&
      (abs(angle_init[1] - angle_ref[1]) < 0.001) &&
      (abs(angle_init[2] - angle_ref[2]) < 0.001) &&
      (abs(angle_init[3] - angle_ref[3]) < 0.001) &&
      (abs(angle_init[4] - angle_ref[4]) < 0.001) &&
      (abs(angle_init[5] - angle_ref[5]) < 0.001)      
      )
  {
    initPoseFlag = false;
    Q_M << 0, 0, 0, 0;
    Q_M_2 << 0, 0, 0, 0;
    Q_angle_d << 0, 0;
    Q_angle_d_2 << 0, 0;
  }

// ROS_INFO("DOING");

}

void TorqJ::PublishCmdNMeasured()
{
  sensor_msgs::JointState joint_cmd;
  sensor_msgs::JointState joint_measured;

  geometry_msgs::Twist first_publisher;
  geometry_msgs::Twist second_publisher;
  // tau_des를 publish
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.position.push_back(angle_safe[0]); 
  joint_cmd.position.push_back(angle_safe[1]); 
  joint_cmd.position.push_back(angle_safe[2]); 
  joint_cmd.position.push_back(angle_safe[3]); 
  joint_cmd.position.push_back(angle_safe[4]); 
  joint_cmd.position.push_back(angle_safe[5]);
//  joint_cmd.position.push_back(gripper_cmd);

  // joint_cmd.velocity.push_back(angle_d[1]); 
  // joint_cmd.velocity.push_back(angle_ref[1]); 
  // joint_cmd.velocity.push_back(angle_ref[2]); 
  // joint_cmd.velocity.push_back(angle_ref[3]); 
  // joint_cmd.velocity.push_back(angle_ref[4]); 
  // joint_cmd.velocity.push_back(angle_ref[5]); 

  // joint_cmd.effort.push_back(angle_measured[0]); 
  // joint_cmd.effort.push_back(angle_measured[1]); 
  // joint_cmd.effort.push_back(angle_measured[2]); 
  // joint_cmd.effort.push_back(angle_measured[3]); 
  // joint_cmd.effort.push_back(angle_measured[4]); 
  // joint_cmd.effort.push_back(angle_measured[5]);  

  first_publisher.linear.x = EE_command_vel_limit[0];
  first_publisher.linear.y = EE_command_vel_limit[1];
  first_publisher.linear.z = EE_command_vel_limit[2];
  first_publisher.angular.x = EE_command[0];
  first_publisher.angular.y = EE_command[1];
  first_publisher.angular.z = EE_command[2];

  second_publisher.linear.x = angle_measured[0];
  second_publisher.linear.y = angle_measured[1];
  second_publisher.linear.z = angle_measured[2];
  second_publisher.angular.x = angle_measured[3];
  second_publisher.angular.y = angle_measured[4];
  second_publisher.angular.z = angle_measured[5];


  test_Pub.publish(first_publisher);
  test_Pub2.publish(second_publisher);

  joint_command_pub_.publish(joint_cmd);


  // ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", EE_command[0], EE_command[1], EE_command[2], EE_command[3], EE_command[4], EE_command[5]);
  // ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", FK_pose[0], FK_pose[1], FK_pose[2], FK_pose[3], FK_pose[4], FK_pose[5]);

}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  checkFirstPoseFlag = 0;

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("/dasom/camera_image", 1);
  DasomCam ds_cam(pub, 0);


  //--초기에 콜백이 충분히 돌도록 유도함 --//
  ros::Rate init_rate(1);
  ros::spinOnce();
  init_rate.sleep();
  ros::spinOnce();
  torqJ.angle_ref = torqJ.angle_measured;
  ROS_INFO("Start to initPose!!");
  //--//

  ros::Rate loop_rate(200);
  time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
  safety++;
  //  ds_cam.UpdateCamera(0, 0, 0);

    //---Measure sampling time---//
    time_f = ros::Time::now().toSec();
    time_loop = time_f - time_i;
    time_i = ros::Time::now().toSec();

    if(torqJ.initPoseFlag)
    {
      torqJ.initPoseFunc();
      ros::spinOnce();   


      torqJ.angle_safe_func();
      torqJ.PublishCmdNMeasured();  
         
    }
    else
    {
    torqJ.CommandGenerator(); //
    torqJ.CommandVelocityLimit();
    torqJ.solveInverseKinematics();
    torqJ.DoB();
    torqJ.angle_safe_func();
    torqJ.PublishCmdNMeasured();      
    } 

    // torqJ.CommandGenerator();
    // torqJ.CommandVelocityLimit();
    // torqJ.solveInverseKinematics();
    // torqJ.angle_command = torqJ.angle_ref;
    // torqJ.DoB(); // 괜히 주석처리 복잡하게 하지말고, 디오비 끄고싶으면 편하게 그냥 디오비 func만 주석처리하자.
    // torqJ.angle_safe_func();
    // torqJ.PublishCmdNMeasured();


    // torqJ.ds_wb_->run();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}