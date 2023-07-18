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

#include "two_link/dasom_kdl.h"

DasomKDL::DasomKDL() 
: nh_(""), loop_rate_(200)
{
  // command_pub_ = nh_.advertise<sensor_msgs::JointState>("/goal_EE_position", 100);

  // Initialize other member variables or perform any necessary setup
  initializeRobotLinks();
}

DasomKDL::~DasomKDL()
{
  // Perform any necessary cleanup
  ROS_INFO("Bye!");
  ros::shutdown();
}

void DasomKDL::run()
{
  float cnt__ = 0;
  float sec = 0;
  
  // DasomKDL.initializeRobotLinks();

  while (ros::ok())
  {
    // 관절 상태 초기화 및 MCG 다이나믹스 행렬 계산 등의 작업 수행

    // 여기서 관절 위치(q_), 관절 속도(q_dot_), 관절 가속도(q_dotdot_)를 초기화할 수 있습니다.
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
    {
      q_(i) = sin(cnt__ / 100);         // 관절 위치를 적절히 설정하세요.
      q_dot_(i) = sin(cnt__ / 100 + 1); // 관절 속도를 적절히 설정하세요.
      q_dotdot_(i) = sin(cnt__ / 100 - 1); // 관절 가속도를 적절히 설정하세요.
    }
    
    computeMCGDynamics();

    ros::spinOnce();
    loop_rate_.sleep();

    cnt__++;
    sec = cnt__ / 100;
    ROS_INFO("SEC:%lf", sec);
    
    // Publish the command message
    // command_pub_.publish(msg_);
  }
}

void DasomKDL::initializeRobotLinks()
{
  // 로봇팔의 구성 정보를 직접 작성합니다.
  unsigned int num_joints = 6;  // 관절의 수
  unsigned int num_links = num_joints + 1;  // 링크의 수

  // 각 링크의 길이, 질량, 무게 중심, 관성 모멘트 정보를 설정합니다.
  std::vector<double> link_lengths(num_links);        // 링크 길이
  std::vector<double> link_masses(num_links);         // 링크 질량
  std::vector<KDL::Vector> link_cogs(num_links);       // 링크 무게 중심
  std::vector<KDL::RotationalInertia> link_inertias(num_links);  // 링크 관성 모멘트
  
  // 각 링크의 길이, 질량, 무게 중심, 관성 모멘트 정보를 초기화합니다.
  for (unsigned int i = 0; i < num_links; ++i)
  {
    link_lengths[i] = 0.158;                   // 링크의 길이를 적절히 설정하세요.
    link_masses[i] = 0.201;                    // 링크의 질량을 적절히 설정하세요.
    link_cogs[i] = KDL::Vector(0.128, 0.072, 0.0); // 링크의 무게 중심을 적절히 설정하세요.
    double ixx = 0.000067648;                        // 관성 모멘트 정보를 적절히 설정하세요.
    double iyy = 0.000340463;
    double izz = 0.000310225;
    double ixy = 0.0;
    double ixz = 0.000001570;
    double iyz = 0.0;
    link_inertias[i] = KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz); // 초기화된 관성 모멘트를 설정합니다.
  }

  // 로봇팔의 체인을 생성합니다.
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    KDL::Vector joint_axis(0.0, 0.0, 1.0);  // 관절 축을 설정합니다.
    KDL::Joint joint(KDL::Joint::RotZ);     // 관절 유형을 설정합니다. 여기서는 회전 관절(RotZ)을 사용합니다.
    KDL::Frame frame(KDL::Vector(link_lengths[i], 0.0, 0.0));  // 관절 위치를 설정합니다.
    frame.p = link_cogs[i];  // 링크의 무게 중심을 설정합니다.
    KDL::RigidBodyInertia link_inertia(link_masses[i], link_cogs[i], link_inertias[i]); // 관성 모멘트를 설정합니다.
    kdl_chain_.addSegment(KDL::Segment(joint, frame, link_inertia));
  }

  // 로봇 관절 상태를 설정합니다.
  q_.resize(num_joints);     // 관절 위치
  q_dot_.resize(num_joints); // 관절 속도
  q_dotdot_.resize(num_joints); // 관절 가속도
}

void DasomKDL::computeMCGDynamics()
{
  // MCG 다이나믹스 행렬을 계산하고 출력하는 작업을 수행합니다.

  // MCG 다이나믹스 행렬을 계산하는데 사용할 변수들을 선언합니다.
  KDL::JntSpaceInertiaMatrix H(kdl_chain_.getNrOfJoints());
  KDL::JntArray C(kdl_chain_.getNrOfJoints());
  KDL::JntArray G(kdl_chain_.getNrOfJoints());
  KDL::JntArray q_dotdot_desired(kdl_chain_.getNrOfJoints());

  // 관절 상태 및 원하는 관절 가속도를 설정합니다. (임의로 설정된 값입니다.)
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    q_dotdot_desired(i) = 0.0; // 원하는 관절 가속도를 설정하세요. (임의로 0으로 설정하였습니다.)
  }

  // MCG 다이나믹스 행렬 계산을 위해 ChainDynParam 클래스 객체를 생성합니다.
  KDL::ChainDynParam dyn_param(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81)); // 중력 벡터를 설정합니다.

  // MCG 다이나믹스 행렬을 계산합니다.
  dyn_param.JntToMass(q_, H);
  dyn_param.JntToCoriolis(q_, q_dot_, C);
  dyn_param.JntToGravity(q_, G);

  // 계산된 MCG 다이나믹스 행렬을 출력합니다.
  ROS_INFO("Mass matrix (H):");
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); ++j)
    {
      ROS_INFO("%f\t", H(i, j));
    }
    ROS_INFO("\n");
  }

  ROS_INFO("Coriolis matrix (C):");
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    ROS_INFO("%f\t", C(i));
  }
  ROS_INFO("\n");

  ROS_INFO("Gravity vector (G):");
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    ROS_INFO("%f\t", G(i));
  }
  ROS_INFO("\n");
}


// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/frames_io.hpp>
// #include <ros/ros.h>
// #include <cmath>
// #include <sensor_msgs/JointState.h>
// #include <two_link/dasom_kdl.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "kdl");
//   ros::NodeHandle n;
//   ros::Publisher Commandpub = n.advertise<sensor_msgs::JointState>("/goal_EE_position", 100); // Final Angle Command

//   sensor_msgs::JointState msg;


//   // // 로봇팔의 구성 정보를 직접 작성합니다.
//   // unsigned int num_joints = 6;  // 관절의 수
//   // unsigned int num_links = num_joints + 1;  // 링크의 수

//   // // 각 링크의 길이, 질량, 무게 중심, 관성 모멘트 정보를 설정합니다.
//   // std::vector<double> link_lengths(num_links);        // 링크 길이
//   // std::vector<double> link_masses(num_links);         // 링크 질량
//   // std::vector<KDL::Vector> link_cogs(num_links);       // 링크 무게 중심
//   // std::vector<KDL::RotationalInertia> link_inertias(num_links);  // 링크 관성 모멘트
  
//   // // 각 링크의 길이, 질량, 무게 중심, 관성 모멘트 정보를 초기화합니다.
//   // for (unsigned int i = 0; i < num_links; ++i)
//   // {
//   //   link_lengths[i] = 0.158;                   // 링크의 길이를 적절히 설정하세요.
//   //   link_masses[i] = 0.201;                    // 링크의 질량을 적절히 설정하세요.
//   //   link_cogs[i] = KDL::Vector(0.128, 0.072, 0.0); // 링크의 무게 중심을 적절히 설정하세요.
//   //   double ixx = 0.000067648;                        // 관성 모멘트 정보를 적절히 설정하세요.
//   //   double iyy = 0.000340463;
//   //   double izz = 0.000310225;
//   //   double ixy = 0.0;
//   //   double ixz = 0.000001570;
//   //   double iyz = 0.0;
//   //   link_inertias[i] = KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz); // 초기화된 관성 모멘트를 설정합니다.
//   // }

//   // // 로봇팔의 체인을 생성합니다.
//   // KDL::Chain kdl_chain;
//   // for (unsigned int i = 0; i < num_joints; ++i)
//   // {
//   //   KDL::Vector joint_axis(0.0, 0.0, 1.0);  // 관절 축을 설정합니다.
//   //   KDL::Joint joint(KDL::Joint::RotZ);     // 관절 유형을 설정합니다. 여기서는 회전 관절(RotZ)을 사용합니다.
//   //   KDL::Frame frame(KDL::Vector(link_lengths[i], 0.0, 0.0));  // 관절 위치를 설정합니다.
//   //   frame.p = link_cogs[i];  // 링크의 무게 중심을 설정합니다.
//   //   KDL::RigidBodyInertia link_inertia(link_masses[i], link_cogs[i], link_inertias[i]); // 관성 모멘트를 설정합니다.
//   //   kdl_chain.addSegment(KDL::Segment(joint, frame, link_inertia));
//   // }

//   // // 로봇 관절 상태를 설정합니다.
//   // KDL::JntArray q(num_joints);     // 관절 위치
//   // KDL::JntArray q_dot(num_joints); // 관절 속도
//   // KDL::JntArray q_dotdot(num_joints); // 관절 가속도


//   // ros::Rate loop(300);


//   // float cnt__ = 0;
//   // float sec = 0;
//   // while(ros::ok())
//   // {


//   // // 관절 상태를 초기화합니다.
//   // for (unsigned int i = 0; i < num_joints; ++i)
//   // {
//   //   q(i) = sin(cnt__ / 100);           // 관절 위치를 적절히 설정하세요.
//   //   q_dot(i) = sin(cnt__ / 100 + 1);       // 관절 속도를 적절히 설정하세요.
//   //   q_dotdot(i) = sin(cnt__ / 100 - 1);    // 관절 가속도를 적절히 설정하세요.
//   // }

//   // // MCG 다이나믹스 행렬을 계산합니다.
//   // KDL::ChainDynParam dyn_param(kdl_chain, KDL::Vector(0.0, 0.0, -9.81));  // 중력 벡터를 설정합니다.
//   // KDL::JntSpaceInertiaMatrix H(num_joints);
//   // KDL::JntArray C(num_joints);
//   // KDL::JntArray G(num_joints);
//   // dyn_param.JntToMass(q, H);
//   // dyn_param.JntToCoriolis(q, q_dot, C);
//   // dyn_param.JntToGravity(q, G);

//   // // 계산된 MCG 다이나믹스 행렬을 출력합니다.
//   // std::cout << "Mass matrix (H):" << std::endl;
//   // for (unsigned int i = 0; i < num_joints; ++i)
//   // {
//   //   for (unsigned int j = 0; j < num_joints; ++j)
//   //   {
//   //       std::cout << H(i, j) << "\t";
//   //   }
//   //   std::cout << std::endl;
//   // }

//   // std::cout << "Coriolis matrix (C):" << std::endl;
//   // for (unsigned int i = 0; i < num_joints; ++i)
//   // {
//   //   std::cout << C(i) << "\t";
//   // }
//   // std::cout << std::endl;

//   // std::cout << "Gravity vector (G):" << std::endl;
//   // for (unsigned int i = 0; i < num_joints; ++i)
//   // {
//   //   std::cout << G(i) << "\t";
//   // }
//   // std::cout << std::endl;

//   // loop.sleep();

//   // cnt__++;
//   // sec = cnt__ / 100;
//   // ROS_INFO("SEC:%lf", sec);



//   // Commandpub.publish(msg);

//   // }
//   return 0;
// }