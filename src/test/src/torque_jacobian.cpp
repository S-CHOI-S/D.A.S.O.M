#include "test/torque_jacobian.h"

TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  X_gain = node_handle_.param<int>("X_gain",1);
  Y_gain = node_handle_.param<int>("Y_gain",1);
  
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();
  initClient();

  ROS_INFO("TorqJ node start");
}

TorqJ::~TorqJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void TorqJ::initPublisher()
{
  // dasom_command_velocity_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DasomDynamixel>(robot_name_ + "/goal_dynamixel_velocity", 10);
}

void TorqJ::initSubscriber()
{
  joint_states_sub  = node_handle_.subscribe("/joint_states", 10, &TorqJ::commandCallback, this);;
  EE_command_sub = node_handle_.subscribe("/goal_dymnamixel_position", 10, &TorqJ::commandCallback, this);  // Command From rqt
  // dasom_joint_states_sub = node_handle_.subscribe(robot_name_ + "/joint_states", 10, &TorqJ::poseCallback, this);
}

void TorqJ::initServer()
{

}

void TorqJ::initClient()
{
  //client = node_handle_.serviceClient<dynamixel_workbench_msgs::EECommand>("/EE_command");
}

// void TorqJ::EEpositionCallback(const dynamixel_workbench_msgs::EECommand::Request &req, dynamixel_workbench_msgs::EECommand::Response &res)
// {
//   X = req.X;
//   Y = req.Y;


// }

void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
// GUI로부터 cmd_position 값 받아서 x'구하기
{
  //////////////////
  command_x_position = msg->position.at(0);
  command_y_position = msg->position.at(1);

	position_x_dot = X_gain * (command_x_position - measured_x_position);
  position_y_dot = Y_gain * (command_y_position - measured_y_position);

  X_dot << position_x_dot,position_y_dot;

  //////////////////
  
  
  // ROS_INFO("command_position_fromGUI = %lf, p_gain = %lf, position_dot = %lf", command_position_fromGUI, p_gain, position_dot);
  // ROS_INFO("x'= %lf, y'= %lf, z'= %lf",position_x_dot, position_y_dot, position_z_dot);
  // ROS_WARN("r'= %lf, p'= %lf, y'= %lf",angle_r_dot, angle_p_dot, angle_yaw_dot);
}

void TorqJ::poseCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  EE_position = EE_pos(msg->position.at(0),msg->position.at(1));

  JT = Jacobian(msg->position.at(0),msg->position.at(1)).transpose();
}

void TorqJ::calc_qdot()
{
  

  JT.resize(6,6);
  X_dot.resize(6,1);
  
  q_dot = JT * X_dot;
}

// void TorqJ::Callback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg)
// // 현재 position & velocity 값 받아오기 // FK?
// {
//   // measured_x_position = msg->pose.position.x;
//   // measured_y_position = msg->pose.position.y;
//   // measured_z_position = msg->pose.position.z;
//   // measured_r_angle = msg->pose.orientation.x;
//   // measured_p_angle = msg->pose.orientation.y;
//   // measured_yaw_angle = msg->pose.orientation.z;
// 	// measured_velocity = msg->velocity.at(0);

//   EE_position = EE_pos(msg->position.at(0),msg->position.at(1),msg->position.at(2),
//                        msg->position.at(3),msg->position.at(4),msg->position.at(5));

//   // orientation이 이상한 것 같은데..?
//   EE_orient = EE_orientation(msg->position.at(0),msg->position.at(1),msg->position.at(2),
//                              msg->position.at(3),msg->position.at(4),msg->position.at(5));

//   JT = Jacobian(msg->position.at(0),msg->position.at(1),msg->position.at(2),
//                 msg->position.at(3),msg->position.at(4),msg->position.at(5)).transpose();

//   measured_x_position = EE_position[0];
//   measured_y_position = EE_position[1];
//   measured_z_position = EE_position[2];
//   measured_r_angle = EE_orient[0];
//   measured_p_angle = EE_orient[1];
//   measured_yaw_angle = EE_orient[2]; // 값이 (-) 반대로 나옴

//   // std::cout<<EE_position<<std::endl<<"---------------------------------------"<<std::endl;
//   std::cout<<EE_orient*180/PI<<std::endl<<"---------------------------------------"<<std::endl;
// }

// void TorqJ::calc_qdot()
// {
//   dynamixel_workbench_msgs::DasomDynamixel dynamixel_;
  
//   JT.resize(6,6);
//   X_dot.resize(6,1);
  
//   q_dot = JT * X_dot;

//   // print!
//   // std::cout<<q_dot<<std::endl<<"---------------------------------------"<<std::endl;
 
//   dynamixel_.header.stamp = ros::Time::now(); // ADD
  
//   for (int index = 0; index < 6; index++) // ADD
//   {
//     std::stringstream id_num;
//     id_num << "id_" << index + 1;

//     dynamixel_.name.push_back(id_num.str());

//     dynamixel_.pose.position.x = EE_position[0];
//     dynamixel_.pose.position.y = EE_position[1];
//     dynamixel_.pose.position.z = EE_position[2];

//     dynamixel_.pose.orientation.x = EE_orient[0]*180/PI;
//     dynamixel_.pose.orientation.y = EE_orient[1]*180/PI;
//     dynamixel_.pose.orientation.z = EE_orient[2]*180/PI;

//     dynamixel_.velocity.push_back(q_dot[index]);
//   }

//   dasom_command_velocity_pub_.publish(dynamixel_);
  
// }

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ TorqJ;

  ros::Rate loop_rate(250); //250
  while (ros::ok())
  {
    // TorqJ.calc_qdot();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}