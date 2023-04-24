#include "test/torque_jacobian.h"

TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  X_gain = node_handle_.param<int>("X_gain",1);
  Y_gain = node_handle_.param<int>("Y_gain",1);

  X_vel_gain = node_handle_.param<int>("X_vel_gain",1);
  Y_vel_gain = node_handle_.param<int>("Y_vel_gain",1);
  
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  V_gain << X_vel_gain, Y_vel_gain;
  // std::cout<<V_gain<<std::endl<<"---------------------------------------"<<std::endl;

  ROS_INFO("TorqJ node start");
}

TorqJ::~TorqJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void TorqJ::initPublisher()
{
  joint_command_pub = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
}

void TorqJ::initSubscriber()
{
  joint_states_sub  = node_handle_.subscribe("/joint_states", 10, &TorqJ::poseCallback, this);
  EE_command_sub = node_handle_.subscribe("/goal_EE_position", 10, &TorqJ::commandCallback, this);
  // dasom_joint_states_sub = node_handle_.subscribe(robot_name_ + "/joint_states", 10, &TorqJ::poseCallback, this);
}

void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
// cmd_position 값 받아서 X'구하기
{
  // X_command, Y_command 값 받아오기
  command_x_position = msg->position.at(0);
  command_y_position = msg->position.at(1);

	position_x_dot = X_gain * (command_x_position - measured_x_position);
  position_y_dot = Y_gain * (command_y_position - measured_y_position);

  X_dot << position_x_dot, position_y_dot;

  //////
  V_dot[0] = V_gain[0] * (X_dot[0] - V_measured[0]);
  V_dot[1] = V_gain[1] * (X_dot[1] - V_measured[1]);

  // V_dot << velocity_x_dot, velocity_y_dot;


  
  // ROS_INFO("command_position_fromGUI = %lf, p_gain = %lf, position_dot = %lf", command_position_fromGUI, p_gain, position_dot);
  // ROS_INFO("x'= %lf, y'= %lf, z'= %lf",position_x_dot, position_y_dot, position_z_dot);
  // ROS_WARN("r'= %lf, p'= %lf, y'= %lf",angle_r_dot, angle_p_dot, angle_yaw_dot);
}

void TorqJ::poseCallback(const sensor_msgs::JointState::ConstPtr &msg)
// 현재 joint angle 값 받아오기
{
  EE_position = EE_pos(msg->position.at(0), msg->position.at(1));

  J = Jacobian(msg->position.at(0),msg->position.at(1));
  JT = J.transpose();

  measured_x_position = EE_position[0];
  measured_y_position = EE_position[1];

  theta_dot << msg->velocity.at(0), msg->velocity.at(1);

  V_measured = J * theta_dot;

  ///////////////////////
  // measured_x_velocity = (measured_x_position - pre_measured_x_position)/0.004;
  // measured_y_velocity = (measured_y_position - pre_measured_y_position)/0.004;

  // pre_measured_x_position = measured_x_position;
  // pre_measured_y_position = measured_y_position;
  ///////////////////////

  // std::cout<<EE_position<<std::endl<<"---------------------------------------"<<std::endl;
  // std::cout<<V_measured<<std::endl<<"---------------------------------------"<<std::endl;
}

void TorqJ::calc_qdot()
{
  sensor_msgs::JointState joint_cmd;

  JT.resize(2,2);
  V_dot.resize(2,1);
  
  tau_des = JT * V_dot;


  // tau_des를 publish
  
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(250); //250
  while (ros::ok())
  {
    torqJ.calc_qdot();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}