#include "test/limit.h"

Limit::Limit()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  ROS_INFO("Limit node start");
}

Limit::~Limit()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void Limit::initPublisher()
{
  dasom_joint_states_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DasomDynamixel>(robot_name_ + "/goal_dynamixel_position", 10);
}

void Limit::initSubscriber()
{
  goal_joint_states_sub_ = node_handle_.subscribe(robot_name_ + "/goal_EE_position", 10, &Limit::poseCallback, this);
}

void Limit::poseCallback(const geometry_msgs::Twist &msg)
{
  x = msg.linear.x;
  y = msg.linear.y;
  z = msg.linear.z;

  q1 = atan2(y,x);

  x2 = x - l4 * cos(phi) * cos(q1);
  y2 = y - l4 * cos(phi) * sin(q1);
  z2 = z - l4 * sin(phi);

  r = sqrt(pow(x2,2) + pow(y2,2));

  cos_q3 = (pow(x2,2) + pow(y2,2) + pow(z2 - l1,2) - pow(l2,2) - pow(l3,2)) / (2 * l2 * l3);
  sin_q3 = sqrt(1 - pow(cos_q3,2));
  q3 = atan2(sin_q3,cos_q3);

  k1 = l2 + l3 * cos_q3;
  k2 = l3 * sin_q3;

  cos_q2 = (k1 * r + k2 * (z2 - l1)) / (pow(k1,2) + pow(k2,2));
  sin_q2 = (k1 * (z2 -l1) - k2 * r) / (pow(k1,2) + pow(k2,2)); // 부호에 따라 elbow up(+) & elbow down(-) 결정 -> 현재 부호 = (+)
  
  q2 = atan2(sin_q2,cos_q2);
  q4 = phi - q2 - q3;
  // ROS_INFO("q1 = %lf, q2 = %lf, q3 = %lf, q4 = %lf", q1, q2, q3, q4);
}

void Limit::goaljointposition() // solve IK
{
  dynamixel_workbench_msgs::DasomDynamixel dynamixel_;

  dynamixel_.header.stamp = ros::Time::now(); // ADD

  for (int index = 0; index < 4; index++) // ADD
  {
    std::stringstream id_num;
    id_num << "id_" << index + 1;

    // std::stringstream angle;
    // angle << "q" << index +1;

    dynamixel_.name.push_back(id_num.str());

    // dynamixel_.position.push_back(angle.str());

    // ROS_INFO("%s",angle.str());

    // dynamixel_.position.push_back(msg->position.at(index));
    // velocity
    // effort
  }

  dynamixel_.position.push_back(q1);
  dynamixel_.position.push_back(q2);
  dynamixel_.position.push_back(q3);
  dynamixel_.position.push_back(q4);

  if (!(-M_PI < q1 && q1 < M_PI))
  {
    ROS_ERROR("Error[001] : Joint1 is limited!\n");
    dynamixel_.flag.push_back(1);
  }
  else dynamixel_.flag.push_back(0);

  if (!(-M_PI_2 < q2 && q2 < 1.53))
  {
    ROS_ERROR("Error[002] : Joint2 is limited!\n");
    dynamixel_.flag.push_back(1);
  }
  else dynamixel_.flag.push_back(0);
    
  if (!(-M_PI_2 < q3 && q3 < 1.53))
  {
    ROS_ERROR("Error[003] : Joint3 is limited!\n");
    dynamixel_.flag.push_back(1);
  }
  else dynamixel_.flag.push_back(0);

  dynamixel_.flag.push_back(0); // Joint4

  ROS_INFO("q1 = %lf, q2 = %lf, q3 = %lf, q4 = %lf", q1, q2, q3, q4);

  dasom_joint_states_pub_.publish(dynamixel_);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "limit");
  Limit limit;

  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    limit.goaljointposition();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}