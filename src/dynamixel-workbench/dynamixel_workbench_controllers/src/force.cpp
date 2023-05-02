#include "dynamixel_workbench_controllers/force.h"

Force::Force()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  ROS_INFO("Force node start");
}

Force::~Force()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void Force::initPublisher()
{
  estimated_force_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/estimated_force",10);
  // testerpub = node_handle_.advertise<geometry_msgs::Twist>("/tester", 10);
}

void Force::initSubscriber()
{
  joint_effort_sub_ = node_handle_.subscribe("/joint_states", 10, &Force::jointCallback, this);
}

void Force::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // angle 값 callback
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);

  // velocity 값 callback
  velocity_measured[0] = msg->velocity.at(0);
  velocity_measured[1] = msg->velocity.at(1);

  // effort 값 callback
  effort_measured[0] = msg->effort.at(0);
  effort_measured[1] = msg->effort.at(1);
}

void Force::setDynamicsMatrix()
{
  M = Moment(angle_measured[0], angle_measured[1]);
  C = CentrifugalNCoriolis(angle_measured[0], angle_measured[1], velocity_measured[0], velocity_measured[1]);
  G = Gravity(angle_measured[0], angle_measured[1]);

  // std::cout<< M <<std::endl<<"---------------------------------------"<<std::endl;
  // std::cout<< C <<std::endl<<"---------------------------------------"<<std::endl;
  // std::cout<< G <<std::endl<<"---------------------------------------"<<std::endl;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "force");
  Force force;

  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    force.setDynamicsMatrix();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}