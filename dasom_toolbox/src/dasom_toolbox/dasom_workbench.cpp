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

#include "../../include/dasom_toolbox/dasom_workbench.h"

using namespace dasom;

DasomWorkbench::DasomWorkbench()
: nh_(""), priv_nh_("")
{
  // For kinematics
  l1 = priv_nh_.param<double>("l1", 0.05465);
  l2 = priv_nh_.param<double>("l2", 0.1585);
  l3 = priv_nh_.param<double>("l3", 0.099);
  l4 = priv_nh_.param<double>("l4", 0.04233);
  l5 = priv_nh_.param<double>("l5", 0.06975);
  l6 = priv_nh_.param<double>("l6", 0.04233);
  l7 = priv_nh_.param<double>("l7", 0.1);

  m1 = priv_nh_.param<double>("m1", 0.201);
  m2 = priv_nh_.param<double>("m2", 0.201);
  m3 = priv_nh_.param<double>("m3", 0.201);
  m4 = priv_nh_.param<double>("m4", 0.201);
  m5 = priv_nh_.param<double>("m5", 0.201);
  m6 = priv_nh_.param<double>("m6", 0.201);
  m7 = priv_nh_.param<double>("m7", 0.201);

  // For admittance control
  virtual_mass_x = priv_nh_.param<double>("virtual_mass_x", 1);
  virtual_damper_x = priv_nh_.param<double>("virtual_damper_x", 1);
  virtual_spring_x = priv_nh_.param<double>("virtual_spring_x", 1);

  virtual_mass_y = priv_nh_.param<double>("virtual_mass_y", 1);
  virtual_damper_y = priv_nh_.param<double>("virtual_damper_y", 1);
  virtual_spring_y = priv_nh_.param<double>("virtual_spring_y", 1);

  virtual_mass_z = priv_nh_.param<double>("virtual_mass_z", 1);
  virtual_damper_z = priv_nh_.param<double>("virtual_damper_z", 1);
  virtual_spring_z = priv_nh_.param<double>("virtual_spring_z", 1);

  initializeRobotLinks();
  initializeAdmittance();
}

DasomWorkbench::~DasomWorkbench()
{
  ROS_INFO("Bye DasomWorkbench!");
  ros::shutdown();
}

void DasomWorkbench::test()
{
  ROS_INFO("HI! Here is DasomWorkbench!");
}

void DasomWorkbench::KDLrun(Eigen::VectorXd angle, Eigen::VectorXd velocity)
{
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    q_(i) = angle[i];
    q_dot_(i) = velocity[i];
    q_dotdot_(i) = 0;
  }

  computeMCGDynamics();
}

void DasomWorkbench::initializeRobotLinks()
{
  unsigned int num_joints = 7;
  unsigned int num_links = num_joints;
  // unsigned int num_joints = 6;
  // unsigned int num_links = num_joints + 1;

  link_lengths.resize(num_links);
  link_masses.resize(num_links);
  link_cogs.resize(num_links);
  link_inertias.resize(num_links);

  link_lengths = {l1, l2, l3, l4, l5, l6, l7};
  link_masses = {m1, m2, m3, m4, m5, m6, m7};
  link_cogs = {
                1.0e-03 * KDL::Vector(120.79, 0.0, 0.39), // link1
                1.0e-03 * KDL::Vector(120.79, 0.0, 0.39), // link2
                1.0e-03 * KDL::Vector(92.17, -8.86, -0.27), // link3
                1.0e-03 * KDL::Vector(47.97, 0.0, 2.18), // link4
                1.0e-03 * KDL::Vector(2.28, -0.41, 35.44), // link5
                1.0e-03 * KDL::Vector(43.4, 3.68, 38.04), // link6
                1.0e-03 * KDL::Vector(6.27, 53.07, -44.27)  // link7
              };
  // link_inertias = KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz);
  link_inertias = {
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link1
                    1.0e-09 * KDL::RotationalInertia(73214.39, 465871.33, 432808.44, 14.73, 2214.66, 0.08), // link2
                    1.0e-09 * KDL::RotationalInertia(53931.12, 191227.48, 197544.93, -24494.86, -744.82, 197.08), // link3
                    1.0e-09 * KDL::RotationalInertia(5484.39, 11290.33, 13145.13, 0.0, -1779.56, 0.0), // link4
                    1.0e-09 * KDL::RotationalInertia(20230.61, 23201.55, 13599.39, 0.0, 28.67, 0.0), // link5
                    1.0e-09 * KDL::RotationalInertia(49705.97, 72814.73, 39460.09, -4033.85, 17779.66, -3638.54), // link6
                    1.0e-09 * KDL::RotationalInertia(728539.02, 332109.09, 538223.79, -68916.07, 9006.52, -27419.39)  // link7
                  };

  // Set Joint Configuration
  addJointSegmentZ(0,2);
  addJointSegmentX(1,1);
  addJointSegmentX(2,1);
  addJointSegmentY(3,-2);
  addJointSegmentFixed(4,1);
  addJointSegmentX(5,2);
  addJointSegmentX(6,0);

  q_.resize(num_joints);
  q_dot_.resize(num_joints);
  q_dotdot_.resize(num_joints);

  H.resize(kdl_chain_.getNrOfJoints()); // M matrix
  C.resize(kdl_chain_.getNrOfJoints()); // C matrix
  G.resize(kdl_chain_.getNrOfJoints()); // G matrix

  M_matrix.resize(H.rows(), H.columns()); // M matrix
  C_matrix.resize(C.rows()); // C matrix
  G_matrix.resize(G.rows()); // G matrix
}

void DasomWorkbench::addJointSegmentX(int jnt_num, int link_frame)
{
  KDL::Vector joint_axis(1.0, 0.0, 0.0);
  KDL::Joint joint(KDL::Joint::RotX);
  KDL::Frame frame;

  // heading axis
  // 0: X, 1: Y, 2: Z
  if(link_frame == 0) 
  {
    frame = KDL::Frame(KDL::Vector(link_lengths[jnt_num], 0.0, 0.0));
  }
  else if(link_frame == 1) 
  {
    frame = KDL::Frame(KDL::Vector(0.0, link_lengths[jnt_num], 0.0));
  }
  else if(link_frame == 2) 
  {
    frame = KDL::Frame(KDL::Vector(0.0, 0.0, link_lengths[jnt_num]));
  }
  
  frame.p = link_cogs[jnt_num];
  KDL::RigidBodyInertia link_inertia(link_masses[jnt_num], link_cogs[jnt_num], link_inertias[jnt_num]);
  kdl_chain_.addSegment(KDL::Segment(joint, frame, link_inertia));
}

void DasomWorkbench::addJointSegmentY(int jnt_num, int link_frame)
{
  KDL::Vector joint_axis(0.0, 1.0, 0.0);
  KDL::Joint joint(KDL::Joint::RotY);
  KDL::Frame frame;

  // heading axis
  // 0: X, 1: Y, 2: Z
  if(link_frame == 0) 
  {
    frame = KDL::Frame(KDL::Vector(link_lengths[jnt_num], 0.0, 0.0));
  }
  else if(link_frame == 1) 
  {
    frame = KDL::Frame(KDL::Vector(0.0, link_lengths[jnt_num], 0.0));
  }
  else if(link_frame == 2)  
  {
    frame = KDL::Frame(KDL::Vector(0.0, 0.0, link_lengths[jnt_num]));
  }
  
  frame.p = link_cogs[jnt_num];
  KDL::RigidBodyInertia link_inertia(link_masses[jnt_num], link_cogs[jnt_num], link_inertias[jnt_num]);
  kdl_chain_.addSegment(KDL::Segment(joint, frame, link_inertia));
}

void DasomWorkbench::addJointSegmentZ(int jnt_num, int link_frame)
{
  KDL::Vector joint_axis(0.0, 0.0, 1.0);
  KDL::Joint joint(KDL::Joint::RotZ);
  KDL::Frame frame;

  // heading axis
  // 0: X, 1: Y, 2: Z, -2: -Z
  if(link_frame == 0) 
  {
    frame = KDL::Frame(KDL::Vector(link_lengths[jnt_num], 0.0, 0.0));
  }
  else if(link_frame == 1) 
  {
    frame = KDL::Frame(KDL::Vector(0.0, link_lengths[jnt_num], 0.0));
  }
  else if(link_frame == 2)  
  {
    frame = KDL::Frame(KDL::Vector(0.0, 0.0, link_lengths[jnt_num]));
  }
  else if(link_frame == -2)  
  {
    frame = KDL::Frame(KDL::Vector(0.0, 0.0, -link_lengths[jnt_num]));
  }

  frame.p = link_cogs[jnt_num];
  KDL::RigidBodyInertia link_inertia(link_masses[jnt_num], link_cogs[jnt_num], link_inertias[jnt_num]);
  kdl_chain_.addSegment(KDL::Segment(joint, frame, link_inertia));
}

void DasomWorkbench::addJointSegmentFixed(int jnt_num, int link_frame)
{
  KDL::Joint joint(KDL::Joint::None);
  KDL::Frame frame;

  if (link_frame == 0)
  {
    frame = KDL::Frame(KDL::Vector(link_lengths[jnt_num], 0.0, 0.0));
  }
  else if (link_frame == 1)
  {
    frame = KDL::Frame(KDL::Vector(0.0, link_lengths[jnt_num], 0.0));
  }
  else if (link_frame == 2)
  {
    frame = KDL::Frame(KDL::Vector(0.0, 0.0, link_lengths[jnt_num]));
  }
  else if (link_frame == -2)
  {
    frame = KDL::Frame(KDL::Vector(0.0, 0.0, -link_lengths[jnt_num]));
  }

  frame.p = link_cogs[jnt_num];
  KDL::RigidBodyInertia link_inertia(link_masses[jnt_num], link_cogs[jnt_num], link_inertias[jnt_num]);
  kdl_chain_.addSegment(KDL::Segment(joint, frame, link_inertia));
}


void DasomWorkbench::computeMCGDynamics()
{
  KDL::JntArray q_dotdot_desired(kdl_chain_.getNrOfJoints());

  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    q_dotdot_desired(i) = 0.0; // desired joint accel
  }

  KDL::ChainDynParam dyn_param(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  dyn_param.JntToMass(q_, H);
  dyn_param.JntToCoriolis(q_, q_dot_, C);
  dyn_param.JntToGravity(q_, G);

  for (int i = 0; i < H.rows(); ++i) 
  {
    for (int j = 0; j < H.columns(); ++j) 
    {
      M_matrix(i, j) = H(i, j);
    }
  }

  for (int i = 0; i < C.rows(); ++i) 
  {
    C_matrix(i) = C(i);
  }

  for (int i = 0; i < G.rows(); ++i) 
  {
    G_matrix(i) = G(i);
  }

  // 계산된 MCG 다이나믹스 행렬을 출력합니다.
  // ROS_INFO("Mass matrix (H):");
  // for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  // {
  //   for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); ++j)
  //   {
  //     ROS_INFO("%f\t", H(i, j));
  //   }
  //   ROS_INFO("\n");
  // }

  // ROS_INFO("Coriolis matrix (C):");
  // for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  // {
  //   ROS_INFO("%f\t", C(i));
  // }
  // ROS_INFO("\n");

  // ROS_INFO("Gravity vector (G):");
  // for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  // {
  //   ROS_INFO("%f\t", G(i));
  // }
  // ROS_INFO("\n");
}

void DasomWorkbench::initializeAdmittance()
{
  A_x << 0, 1,
      -virtual_spring_x / virtual_mass_x, -virtual_damper_x / virtual_mass_x;

  B_x << 0, 1 / virtual_mass_x;

  C_x << 1, 0;

  D_x << 0, 0;

  A_y << 0, 1,
      -virtual_spring_y / virtual_mass_y, -virtual_damper_y / virtual_mass_y;

  B_y << 0, 1 / virtual_mass_y;

  C_y << 1, 0;

  D_y << 0, 0;

  A_z << 0, 1,
        -virtual_spring_z / virtual_mass_z, -virtual_damper_z / virtual_mass_z;

  B_z << 0, 1 / virtual_mass_z;

  C_z << 1, 0;

  D_z << 0, 0;

  X_from_model_matrix << 0, 0;
  X_dot_from_model_matrix << 0, 0;

  Y_from_model_matrix << 0, 0;
  Y_dot_from_model_matrix << 0, 0;

  Z_from_model_matrix << 0, 0;
  Z_dot_from_model_matrix << 0, 0;
}

double DasomWorkbench::admittanceControlX(double time_loop, double ref, double f_ext)
{
  double X_cmd;
  double X_position_from_model;

  X_dot_from_model_matrix = A_x * X_from_model_matrix + B_x * f_ext;
  X_from_model_matrix = X_from_model_matrix + X_dot_from_model_matrix * time_loop;
  X_position_from_model = X_from_model_matrix[0];

  X_cmd = ref - X_position_from_model;

  return X_cmd;
}

double DasomWorkbench::admittanceControlY(double time_loop, double ref, double f_ext)
{
  double Y_cmd;
  double Y_position_from_model;

  Y_dot_from_model_matrix = A_y * Y_from_model_matrix + B_y * f_ext;
  Y_from_model_matrix = Y_from_model_matrix + Y_dot_from_model_matrix * time_loop;
  Y_position_from_model = Y_from_model_matrix[0];

  Y_cmd = ref - Y_position_from_model;

  return Y_cmd;
}

double DasomWorkbench::admittanceControlZ(double time_loop, double ref, double f_ext)
{
  double Z_cmd;
  double Z_position_from_model;

  Z_dot_from_model_matrix = A_z * Z_from_model_matrix + B_z * f_ext;
  Z_from_model_matrix = Z_from_model_matrix + Z_dot_from_model_matrix * time_loop;
  Z_position_from_model = Z_from_model_matrix[0];

  Z_cmd = ref - Z_position_from_model;

  return Z_cmd;
}

// Forward Kinematics
Eigen::Matrix4d DasomWorkbench::L1(double theta_1)
{
  double cosT = cos(theta_1), sinT = sin(theta_1);

  Eigen::Matrix4d L;
  L <<      cosT,     -sinT,     0,        0,
            sinT,      cosT,     0,        0,
               0,         0,     1,       l1,
               0,         0,     0,        1;
  return L;
}

Eigen::Matrix4d DasomWorkbench::L2(double theta_2)
{
  double cosT = cos(theta_2), sinT = sin(theta_2);

  Eigen::Matrix4d L;
  L <<         1,         0,     0,        0,
               0,      cosT, -sinT,  l2*cosT,
               0,      sinT,  cosT,  l2*sinT,
               0,         0,     0,        1;
  return L;
}

Eigen::Matrix4d DasomWorkbench::L3(double theta_3)
{
  double cosT = cos(theta_3), sinT = sin(theta_3);

  Eigen::Matrix4d L;
  L <<         1,         0,     0,        0,
               0,      cosT, -sinT,  l3*cosT,
               0,      sinT,  cosT,  l3*sinT,
               0,         0,     0,        1;
  return L;
}

Eigen::Matrix4d DasomWorkbench::L4(double theta_4)
{
  double cosT = cos(theta_4), sinT = sin(theta_4);

  Eigen::Matrix4d L;
  L <<      cosT,         0,  sinT, -l4*sinT,
               0,         1,     0,        0,
           -sinT,         0,  cosT, -l4*cosT,
               0,         0,     0,        1;
  return L;
}

Eigen::Matrix4d DasomWorkbench::L5()
{
  Eigen::Matrix4d L;
  L <<         1,         0,     0,        0,
               0,         1,     0,       l5,
               0,         0,     1,        0,
               0,         0,     0,        1;
  return L;
}

Eigen::Matrix4d DasomWorkbench::L6(double theta_5)
{
  double cosT = cos(theta_5), sinT = sin(theta_5);

  Eigen::Matrix4d L;
  L <<      cosT,     -sinT,     0,        0,
            sinT,      cosT,     0,        0,
               0,         0,     1,       l6,
               0,         0,     0,        1;
  return L;
}

Eigen::Matrix4d DasomWorkbench::L7(double theta_6)
{
  double cosT = cos(theta_6), sinT = sin(theta_6);

  Eigen::Matrix4d L;
  L <<         1,         0,     0,       0,
               0,      cosT, -sinT, l7*cosT,
               0,      sinT,  cosT, l7*sinT,
               0,         0,     0,       1;
  return L;
}

Eigen::Matrix3d DasomWorkbench::R03(double theta_1, double theta_2, double theta_3)
{
  Eigen::Matrix4d P1;
  P1 << L1(theta_1);

  Eigen::Matrix4d P2;
  P2 << P1 * L2(theta_2);

  Eigen::Matrix4d P3;
  P3 << P2 * L3(theta_3);

  Eigen::Matrix3d R03;
  R03 << P3(0,0), P3(0,1), P3(0,2),
         P3(1,0), P3(1,1), P3(1,2),
         P3(2,0), P3(2,1), P3(2,2);

  return R03;
}

Eigen::MatrixXd DasomWorkbench::EE_pose(Eigen::VectorXd measured_angle)
{
  double cos1 = cos(measured_angle[0]), sin1 = sin(measured_angle[0]);
  double cos2 = cos(measured_angle[1]), sin2 = sin(measured_angle[1]);
  double cos3 = cos(measured_angle[2]), sin3 = sin(measured_angle[2]);
  double cos4 = cos(measured_angle[3]), sin4 = sin(measured_angle[3]);
  double cos5 = cos(measured_angle[4]), sin5 = sin(measured_angle[4]);
  double cos6 = cos(measured_angle[5]), sin6 = sin(measured_angle[5]);

  double r11 = cos(measured_angle[4])*(cos(measured_angle[0])*cos(measured_angle[3]) - sin(measured_angle[3])*(cos(measured_angle[1])*sin(measured_angle[0])*sin(measured_angle[2]) + cos(measured_angle[2])*sin(measured_angle[0])*sin(measured_angle[1]))) + sin(measured_angle[4])*(sin(measured_angle[0])*sin(measured_angle[1])*sin(measured_angle[2]) - cos(measured_angle[1])*cos(measured_angle[2])*sin(measured_angle[0]));
  double r12 = sin(measured_angle[5])*(cos(measured_angle[0])*sin(measured_angle[3]) + cos(measured_angle[3])*(cos(measured_angle[1])*sin(measured_angle[0])*sin(measured_angle[2]) + cos(measured_angle[2])*sin(measured_angle[0])*sin(measured_angle[1]))) - cos(measured_angle[5])*(sin(measured_angle[4])*(cos(measured_angle[0])*cos(measured_angle[3]) - sin(measured_angle[3])*(cos(measured_angle[1])*sin(measured_angle[0])*sin(measured_angle[2]) + cos(measured_angle[2])*sin(measured_angle[0])*sin(measured_angle[1]))) - cos(measured_angle[4])*(sin(measured_angle[0])*sin(measured_angle[1])*sin(measured_angle[2]) - cos(measured_angle[1])*cos(measured_angle[2])*sin(measured_angle[0])));
  double r13 = cos(measured_angle[5])*(cos(measured_angle[0])*sin(measured_angle[3]) + cos(measured_angle[3])*(cos(measured_angle[1])*sin(measured_angle[0])*sin(measured_angle[2]) + cos(measured_angle[2])*sin(measured_angle[0])*sin(measured_angle[1]))) + sin(measured_angle[5])*(sin(measured_angle[4])*(cos(measured_angle[0])*cos(measured_angle[3]) - sin(measured_angle[3])*(cos(measured_angle[1])*sin(measured_angle[0])*sin(measured_angle[2]) + cos(measured_angle[2])*sin(measured_angle[0])*sin(measured_angle[1]))) - cos(measured_angle[4])*(sin(measured_angle[0])*sin(measured_angle[1])*sin(measured_angle[2]) - cos(measured_angle[1])*cos(measured_angle[2])*sin(measured_angle[0])));
  double r21 = cos(measured_angle[4])*(cos(measured_angle[3])*sin(measured_angle[0]) + sin(measured_angle[3])*(cos(measured_angle[0])*cos(measured_angle[1])*sin(measured_angle[2]) + cos(measured_angle[0])*cos(measured_angle[2])*sin(measured_angle[1]))) - sin(measured_angle[4])*(cos(measured_angle[0])*sin(measured_angle[1])*sin(measured_angle[2]) - cos(measured_angle[0])*cos(measured_angle[1])*cos(measured_angle[2]));
  double r22 = sin(measured_angle[5])*(sin(measured_angle[0])*sin(measured_angle[3]) - cos(measured_angle[3])*(cos(measured_angle[0])*cos(measured_angle[1])*sin(measured_angle[2]) + cos(measured_angle[0])*cos(measured_angle[2])*sin(measured_angle[1]))) - cos(measured_angle[5])*(sin(measured_angle[4])*(cos(measured_angle[3])*sin(measured_angle[0]) + sin(measured_angle[3])*(cos(measured_angle[0])*cos(measured_angle[1])*sin(measured_angle[2]) + cos(measured_angle[0])*cos(measured_angle[2])*sin(measured_angle[1]))) + cos(measured_angle[4])*(cos(measured_angle[0])*sin(measured_angle[1])*sin(measured_angle[2]) - cos(measured_angle[0])*cos(measured_angle[1])*cos(measured_angle[2])));
  double r23 = cos(measured_angle[5])*(sin(measured_angle[0])*sin(measured_angle[3]) - cos(measured_angle[3])*(cos(measured_angle[0])*cos(measured_angle[1])*sin(measured_angle[2]) + cos(measured_angle[0])*cos(measured_angle[2])*sin(measured_angle[1]))) + sin(measured_angle[5])*(sin(measured_angle[4])*(cos(measured_angle[3])*sin(measured_angle[0]) + sin(measured_angle[3])*(cos(measured_angle[0])*cos(measured_angle[1])*sin(measured_angle[2]) + cos(measured_angle[0])*cos(measured_angle[2])*sin(measured_angle[1]))) + cos(measured_angle[4])*(cos(measured_angle[0])*sin(measured_angle[1])*sin(measured_angle[2]) - cos(measured_angle[0])*cos(measured_angle[1])*cos(measured_angle[2])));
  double r31 = sin(measured_angle[4])*(cos(measured_angle[1])*sin(measured_angle[2]) + cos(measured_angle[2])*sin(measured_angle[1])) - cos(measured_angle[4])*sin(measured_angle[3])*(cos(measured_angle[1])*cos(measured_angle[2]) - sin(measured_angle[1])*sin(measured_angle[2]));

  double alpha = atan2(r21,r11);

  Eigen::MatrixXd EE_position(3,1);

  EE_position << 
  // X
  l5*(sin1*sin2*sin3 - cos2*cos3*sin1) + l6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - l4*cos4*(cos2*sin1*sin3 + cos3*sin1*sin2) - l7*cos6*(sin5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) - l2*cos2*sin1 - l4*cos1*sin4 + l7*sin6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - l3*cos2*cos3*sin1 + l3*sin1*sin2*sin3,
  // Y
  l6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l5*(cos1*sin2*sin3 - cos1*cos2*cos3) + l2*cos1*cos2 - l4*sin1*sin4 + l7*sin6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l7*cos6*(sin5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + cos5*(cos1*sin2*sin3 - cos1*cos2*cos3)) + l4*cos4*(cos1*cos2*sin3 + cos1*cos3*sin2) + l3*cos1*cos2*cos3 - l3*cos1*sin2*sin3,
  // Z
  l1 + l5*(cos2*sin3 + cos3*sin2) + l2*sin2 - l4*cos4*(cos2*cos3 - sin2*sin3) + l6*cos4*(cos2*cos3 - sin2*sin3) + l3*cos2*sin3 + l3*cos3*sin2 + l7*cos6*(cos5*(cos2*sin3 + cos3*sin2) + sin4*sin5*(cos2*cos3 - sin2*sin3)) + l7*cos4*sin6*(cos2*cos3 - sin2*sin3);

  Eigen::MatrixXd EE_Orientation(3,1);
  
  EE_Orientation << 
  // roll
  atan2(sin(alpha)*r13-cos(alpha)*r23,-sin(alpha)*r12+cos(alpha)*r22),
  // pitch
  atan2(-r31,cos(alpha)*r11+sin(alpha)*r21),				// limit pitch direction
  // yaw
  alpha;

  Eigen::MatrixXd EE_pose(6,1);

  EE_pose <<
  EE_position(0,0),
  EE_position(1,0),
  EE_position(2,0),
  EE_Orientation(0,0),
  EE_Orientation(1,0),
  EE_Orientation(2,0);

  return EE_pose;
}

// Jacobian
Eigen::MatrixXd DasomWorkbench::Jacobian(Eigen::VectorXd measured_angle)
{
  double cos1 = cos(measured_angle[0]), sin1 = sin(measured_angle[0]);
  double cos2 = cos(measured_angle[1]), sin2 = sin(measured_angle[1]);
  double cos3 = cos(measured_angle[2]), sin3 = sin(measured_angle[2]);
  double cos4 = cos(measured_angle[3]), sin4 = sin(measured_angle[3]);
  double cos5 = cos(measured_angle[4]), sin5 = sin(measured_angle[4]);
  double cos6 = cos(measured_angle[5]), sin6 = sin(measured_angle[5]);
  double sin23 = sin(measured_angle[1] + measured_angle[2]);
  double cos23 = cos(measured_angle[1] + measured_angle[2]);
  double cos45 = cos(measured_angle[3] + measured_angle[4]);
  double sin46 = sin(measured_angle[3] + measured_angle[5]);
  double cos4m5 = cos(measured_angle[3] - measured_angle[4]);
  double sin4m6 = sin(measured_angle[3] - measured_angle[5]);

  Eigen::MatrixXd J(6,6);

  J <<
  // 1X1
  l5*(cos1*sin2*sin3 - cos1*cos2*cos3) + l6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l2*cos1*cos2 - l4*sin1*sin4 - l7*sin6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + l7*cos6*(sin5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + cos5*(cos1*sin2*sin3 - cos1*cos2*cos3)) + l4*cos4*(cos1*cos2*sin3 + cos1*cos3*sin2) - l3*cos1*cos2*cos3 + l3*cos1*sin2*sin3,
  // 1X2
  sin1*(l5*sin23 + l2*sin2 + l4*cos23*cos4 - l6*cos23*cos4 + l3*cos2*sin3 + l3*cos3*sin2 + l7*cos23*cos4*sin6 + l7*sin23*cos5*cos6 + l7*cos23*cos6*sin4*sin5),
  // 1X3
  sin1*(l5*sin23 + l4*cos23*cos4 - l6*cos23*cos4 + l3*cos2*sin3 + l3*cos3*sin2 + l7*cos23*cos4*sin6 + l7*sin23*cos5*cos6 + l7*cos23*cos6*sin4*sin5),
  // 1X4
  l4*cos1*cos4 - l4*sin4*(cos2*sin1*sin3 + cos3*sin1*sin2) - l6*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + l7*sin6*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + l7*cos6*sin5*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)),
  // 1X5
  -l7*cos6*(cos5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + sin5*(sin1*sin2*sin3 - cos2*cos3*sin1)),
  // 1X6
  l7*sin6*(sin5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) + l7*cos6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)),
  // 2X1
  l5*(sin1*sin2*sin3 - cos2*cos3*sin1) - l6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + l4*cos4*(cos2*sin1*sin3 + cos3*sin1*sin2) - l7*cos6*(sin5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) - l2*cos2*sin1 + l4*cos1*sin4 + l7*sin6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - l3*cos2*cos3*sin1 + l3*sin1*sin2*sin3,
  // 2X2
  -cos1*(l5*sin23 + l2*sin2 + l4*cos23*cos4 - l6*cos23*cos4 + l3*cos2*sin3 + l3*cos3*sin2 + l7*cos23*cos4*sin6 + l7*sin23*cos5*cos6 + l7*cos23*cos6*sin4*sin5),
  // 2X3
  -cos1*(l5*sin23 + l4*cos23*cos4 - l6*cos23*cos4 + l3*cos2*sin3 + l3*cos3*sin2 + l7*cos23*cos4*sin6 + l7*sin23*cos5*cos6 + l7*cos23*cos6*sin4*sin5),
  // 2X4
  l4*sin4*(cos1*cos2*sin3 + cos1*cos3*sin2) - l6*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + l4*cos4*sin1 + l7*sin6*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + l7*cos6*sin5*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)),
  // 2X5
  -l7*cos6*(cos5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - sin5*(cos1*sin2*sin3 - cos1*cos2*cos3)),
  // 2X6
  l7*cos6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + l7*sin6*(sin5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + cos5*(cos1*sin2*sin3 - cos1*cos2*cos3)),
  // 3X1
  0,
  // 3X2
  l3*cos23 + l5*cos23 + l2*cos2 - (l7*sin23*sin46)/2 - l4*sin23*cos4 + l6*sin23*cos4 + (l7*sin4m6*sin23)/2 + l7*cos6*((cos45*sin23)/2 - (cos4m5*sin23)/2 + cos23*cos5),
  // 3X3
  l3*cos23 + l5*cos23 - l4*sin23*cos4 + l6*sin23*cos4 + l7*cos23*cos5*cos6 - l7*sin23*cos4*sin6 - l7*sin23*cos6*sin4*sin5,
  // 3X4
  -cos23*(l4*sin4 - l6*sin4 + l7*sin4*sin6 -l7*cos4*cos6*sin5),
  // 3X5
  -l7*cos6*(sin23*sin5 -cos23*cos5*sin4),
  // 3X6
  l7*cos23*cos4*cos6 - l7*sin23*cos5*sin6 - l7*cos23*sin4*sin5*sin6,
  // 4X1
  0,
  // 4X2
  cos1,
  // 4X3
  cos1,
  // 4X4
  -cos23*sin1,
  // 4X5
  cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2),
  // 4X6
  cos5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + sin5*(sin1*sin2*sin3 - cos2*cos3*sin1),
  // 5X1
  0,
  // 5X2
  sin1,
  // 5X3
  sin1,
  // 5X4
  cos23*cos1,
  // 5X5
  sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2),
  // 5X6
  cos5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - sin5*(cos1*sin2*sin3 - cos1*cos2*cos3),
  // 6X1
  1,
  // 6X2
  0,
  // 6X3
  0,
  // 6X4
  sin23,
  // 6X5
  cos23*cos4,
  // 6X6
  sin23*sin5 - cos23*cos5*sin4;

  return J;
}

// Inverse Kinematics
Eigen::Matrix3d DasomWorkbench::CmdOrientation(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d R;

  R <<
  // 1X1
  cos(pitch)*cos(yaw),
  // 1X2
  cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw),
  // 1X3
  sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch),
  // 2X1
  cos(pitch)*sin(yaw),
  // 2X2
  cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw),
  // 2X3
  cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll),
  // 3X1
  -sin(pitch),
  // 3X2
  cos(pitch)*sin(roll),
  // 3X3
  cos(pitch)*cos(roll);

  return R;
}

Eigen::VectorXd DasomWorkbench::InverseKinematics(Eigen::VectorXd EndEffector_cmd)
{
  double X = EndEffector_cmd[0];
  double Y = EndEffector_cmd[1];
  double Z = EndEffector_cmd[2];
  double r = EndEffector_cmd[3];
  double p = EndEffector_cmd[4];
  double y = EndEffector_cmd[5];

  double theta1;
  double theta2;
  double D_theta2;
  double theta3;
  double D_theta3;
  double theta4;
  double theta5;
  double theta6;
  double r2;
  double R36_r11;
  double R36_r21;
  double R36_r22;
  double R36_r23;
  double R36_r31;

  Eigen::Vector3d Wrist_Position;

  Wrist_Position <<
  // 1X1
  X - l7 * CmdOrientation(r,p,y)(0,1),
  // 2X1
  Y - l7 * CmdOrientation(r,p,y)(1,1),
  // 3X1
  Z - l7 * CmdOrientation(r,p,y)(2,1);

  r2 = sqrt(pow(Wrist_Position[0],2) + pow(Wrist_Position[1],2) + pow((Wrist_Position[2] - l1),2));

  theta1 = atan2(Wrist_Position[1], Wrist_Position[0]) - PI / 2;

  D_theta2 = (pow(l2,2) + pow(r2,2) - pow((l3 + l5),2)) / (2 * l2 * r2);

  theta2 = atan2(Wrist_Position[2] - l1, sqrt(pow(Wrist_Position[0],2) + pow(Wrist_Position[1],2)))
         + atan2(sqrt(1-pow(D_theta2,2)),D_theta2); // sign

  D_theta3 = (pow(l2,2) + pow((l3 + l5),2) - pow(r2,2)) / (2 * l2 * (l3 + l5));
  theta3 = -(PI - atan2(sqrt(1 - pow(D_theta3,2)), D_theta3)); // sign
  
  Eigen::Matrix3d R36;
  R36 = R03(theta1, theta2, theta3).transpose() * CmdOrientation(r,p,y);

  R36_r11 = R36(0,0);
  R36_r21 = R36(1,0);
  R36_r22 = R36(1,1);
  R36_r23 = R36(1,2);
  R36_r31 = R36(2,0);

  theta4 = atan2(-R36_r31, R36_r11); // pitch

  theta5 = atan2(R36_r21*cos(theta4), R36_r11); // yaw

  theta6 = atan2(-R36_r23, R36_r22); // roll

  Eigen::VectorXd theta(6);
  theta << theta1, theta2, theta3, theta4, theta5, theta6;

  return theta;
}