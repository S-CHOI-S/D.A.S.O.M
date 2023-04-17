#include "test/velocity_jacobian.h"

VelJ::VelJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  p_gain = node_handle_.param<int>("p_gain",1);
  i_gain = node_handle_.param<int>("i_gain",1);
  d_gain = node_handle_.param<int>("d_gain",1);
  
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  ROS_INFO("VelJ node start");
}

VelJ::~VelJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void VelJ::initPublisher()
{
  dasom_command_velocity_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DasomDynamixel>(robot_name_ + "/goal_dynamixel_velocity", 10);
}

void VelJ::initSubscriber()
{
  dasom_command_sub = node_handle_.subscribe(robot_name_ + "/goal_position", 10, &VelJ::commandCallback, this);  // Command From rqt
  dasom_joint_states_sub = node_handle_.subscribe(robot_name_ + "/joint_states", 10, &VelJ::poseCallback, this);
}

void VelJ::commandCallback(const geometry_msgs::Twist &msg)
// GUI로부터 cmd_position 값 받아서 x'구하기
{
  command_x_position_fromGUI = msg.linear.x;
  command_y_position_fromGUI = msg.linear.y;
  command_z_position_fromGUI = msg.linear.z;
  command_r_angle_fromGUI = msg.angular.x;
  command_p_angle_fromGUI = msg.angular.y;
  command_yaw_angle_fromGUI = msg.angular.z;

	position_x_dot = p_gain * (command_x_position_fromGUI - measured_x_position);
  position_y_dot = i_gain * (command_y_position_fromGUI - measured_y_position);
  position_z_dot = d_gain * (command_z_position_fromGUI - measured_z_position);
  angle_r_dot = p_gain * (command_r_angle_fromGUI - measured_r_angle);
  angle_p_dot = i_gain * (command_p_angle_fromGUI - measured_p_angle);
  angle_yaw_dot = d_gain * (command_yaw_angle_fromGUI - measured_yaw_angle);

  X_dot << position_x_dot,position_y_dot,position_z_dot,angle_r_dot,angle_p_dot,angle_yaw_dot;
  
  // ROS_INFO("command_position_fromGUI = %lf, p_gain = %lf, position_dot = %lf", command_position_fromGUI, p_gain, position_dot);
  ROS_INFO("x'= %lf, y'= %lf, z'= %lf",position_x_dot, position_y_dot, position_z_dot);
  ROS_WARN("r'= %lf, p'= %lf, y'= %lf",angle_r_dot, angle_p_dot, angle_yaw_dot);
}

void VelJ::poseCallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg)
// 현재 position & velocity 값 받아오기 // FK?
{
  // measured_x_position = msg->pose.position.x;
  // measured_y_position = msg->pose.position.y;
  // measured_z_position = msg->pose.position.z;
  // measured_r_angle = msg->pose.orientation.x;
  // measured_p_angle = msg->pose.orientation.y;
  // measured_yaw_angle = msg->pose.orientation.z;
	// measured_velocity = msg->velocity.at(0);

  EE_position = EE_pos(msg->position.at(0),msg->position.at(1),msg->position.at(2),
                       msg->position.at(3),msg->position.at(4),msg->position.at(5));

  // orientation이 이상한 것 같은데..?
  EE_orient = EE_orientation(msg->position.at(0),msg->position.at(1),msg->position.at(2),
                             msg->position.at(3),msg->position.at(4),msg->position.at(5));

  JT = Jacobian(msg->position.at(0),msg->position.at(1),msg->position.at(2),
                msg->position.at(3),msg->position.at(4),msg->position.at(5)).transpose();

  measured_x_position = EE_position[0];
  measured_y_position = EE_position[1];
  measured_z_position = EE_position[2];
  measured_r_angle = EE_orient[0];
  measured_p_angle = EE_orient[1];
  measured_yaw_angle = EE_orient[2]; // 값이 (-) 반대로 나옴

  // std::cout<<EE_position<<std::endl<<"---------------------------------------"<<std::endl;
  std::cout<<EE_orient*180/PI<<std::endl<<"---------------------------------------"<<std::endl;
}

void VelJ::calc_qdot()
{
  dynamixel_workbench_msgs::DasomDynamixel dynamixel_;
  
  JT.resize(6,6);
  X_dot.resize(6,1);
  
  q_dot = JT * X_dot;

  // print!
  // std::cout<<q_dot<<std::endl<<"---------------------------------------"<<std::endl;
 
  dynamixel_.header.stamp = ros::Time::now(); // ADD
  
  for (int index = 0; index < 6; index++) // ADD
  {
    std::stringstream id_num;
    id_num << "id_" << index + 1;

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.pose.position.x = EE_position[0];
    dynamixel_.pose.position.y = EE_position[1];
    dynamixel_.pose.position.z = EE_position[2];

    dynamixel_.pose.orientation.x = EE_orient[0]*180/PI;
    dynamixel_.pose.orientation.y = EE_orient[1]*180/PI;
    dynamixel_.pose.orientation.z = EE_orient[2]*180/PI;

    dynamixel_.velocity.push_back(q_dot[index]);
  }

  dasom_command_velocity_pub_.publish(dynamixel_);
  
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "VelJ");
  VelJ velj;

  ros::Rate loop_rate(250); //250
  while (ros::ok())
  {
    velj.calc_qdot();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}