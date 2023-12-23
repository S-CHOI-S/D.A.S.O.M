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

  virtual_mass_x = 10000;
  virtual_damper_x = 10000;
  virtual_spring_x = 10000;

  virtual_mass_y = 10000;
  virtual_damper_y = 10000;
  virtual_spring_y = 10000;

  virtual_mass_z = 10000;
  virtual_damper_z = 10000;
  virtual_spring_z = 10000;

  virtual_damper_DK_x = 10000;
  virtual_spring_DK_x = 10000;

  virtual_damper_DK_y = 10000;
  virtual_spring_DK_y = 10000;

  virtual_damper_DK_z = 10000;
  virtual_spring_DK_z = 10000;


  X_from_model_matrix << 0, 0;
  X_dot_from_model_matrix << 0, 0;

  Y_from_model_matrix << 0, 0;
  Y_dot_from_model_matrix << 0, 0;

  Z_from_model_matrix << 0, 0;
  Z_dot_from_model_matrix << 0, 0;

  initializeRobotLinks();
  initializeAdmittance();
  initializeAdmittanceDK();
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
  unsigned int num_joints = 6;
  unsigned int num_links = num_joints;

  link_lengths.resize(num_links);
  link_masses.resize(num_links);
  link_cogs.resize(num_links);
  link_inertias.resize(num_links);

  link_lengths = {
                   KDL::Vector(0.0,0.0,l1), // link1
                   KDL::Vector(0.0,l2,0.0), // link2
                   KDL::Vector(0.0,l3,0.0), // link3
                   KDL::Vector(0.0,l5,-l4), // link4
                   KDL::Vector(0.0,0.0,l6), // link5
                   KDL::Vector(0.0,l7,0.0)  // link6
                 };
  link_masses = {m1, m2, m3, m4, m5, m6};
  link_cogs = {
                KDL::Vector(0,0,0.027325), // link1
                KDL::Vector(0.00039,-0.03411,0.0), // link2
                KDL::Vector(-0.00001,-0.06095,0.00226), // link3
                KDL::Vector(-0.00033,-0.05253,0.02129), // link4
                KDL::Vector(0.0434,0.00368,-0.02326), // link5
                KDL::Vector(-0.04427,-0.06193,-0.00627)  // link6
              };
  // link_inertias = KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz);
  link_inertias = {
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link1
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link2
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link3
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link4
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link5
                    1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0)  // link6
                  };

  // Set Joint Configuration
  // 0.0
  kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(link_lengths[0]),
                                     KDL::RigidBodyInertia(m1, link_cogs[0], link_inertias[0])));
  // 0.0 0.263367
  kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(link_lengths[1]),
                                     KDL::RigidBodyInertia(m2, link_cogs[1], link_inertias[1])));
  // 0.0 0.577486	0.117183	
  kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(link_lengths[2]),
                                     KDL::RigidBodyInertia(m3, link_cogs[2], link_inertias[2])));
  // 0.0 0.641148	0.146350 0.000080
  kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(link_lengths[3]),
                                     KDL::RigidBodyInertia(m4, link_cogs[3], link_inertias[3])));
  // 0.0 0.991254	0.329760 -0.046625 0.000000	
  kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(link_lengths[4]),
                                     KDL::RigidBodyInertia(m5, link_cogs[4], link_inertias[4])));
  // 0.0 2.110560	0.986372 0.085612 0.000000 0.158523	
  kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(link_lengths[5]),
                                     KDL::RigidBodyInertia(m6, link_cogs[5], link_inertias[5])));
 
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

  // 계산된 MCG 다이나믹스 행렬을 출력합니다.
  // ROS_INFO("Mass matrix (H):");
  // for (unsigned int i = 0; i < H.rows(); ++i)
  // {
  //   for (unsigned int j = 0; j < H.columns(); ++j)
  //   {
  //     ROS_INFO("%f\t", M_matrix(i, j));
  //   }
  //   ROS_INFO("\n");
  // }

  // ROS_INFO("Coriolis matrix (C):");
  // for (unsigned int i = 0; i < C.rows(); ++i)
  // {
  //   ROS_INFO("%f\t", C_matrix(i));
  // }
  // ROS_INFO("\n");

  // ROS_INFO("Gravity vector (G):");
  // for (unsigned int i = 0; i < G.rows(); ++i)
  // {
  //   ROS_INFO("%f\t", G_matrix(i));
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
}

void DasomWorkbench::initializeAdmittanceDK()
{
  X_position_from_model_DK = 0;
  X_position_dot_from_model_DK = 0;

  Y_position_from_model_DK = 0;
  Y_position_dot_from_model_DK = 0;

  Z_position_from_model_DK = 0;
  Z_position_dot_from_model_DK = 0;
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


double DasomWorkbench::admittanceControlDK_X(double time_loop, double ref, double f_ext)
{
  double X_cmd;
  
	X_position_dot_from_model_DK = f_ext/virtual_damper_DK_x - virtual_spring_DK_x/virtual_damper_DK_x * X_position_from_model_DK;
  X_position_from_model_DK = X_position_from_model_DK + X_position_dot_from_model_DK * time_loop;

  X_cmd = ref - X_position_from_model_DK;
  return X_cmd;
}

double DasomWorkbench::admittanceControlDK_Y(double time_loop, double ref, double f_ext)
{
  double Y_cmd;

	Y_position_dot_from_model_DK = f_ext/virtual_damper_DK_y - virtual_spring_DK_y/virtual_damper_DK_y * Y_position_from_model_DK;
  Y_position_from_model_DK = Y_position_from_model_DK + Y_position_dot_from_model_DK * time_loop;

  Y_cmd = ref - Y_position_from_model_DK;
  return Y_cmd;
}

double DasomWorkbench::admittanceControlDK_Z(double time_loop, double ref, double f_ext)
{
  double Z_cmd;

	Z_position_dot_from_model_DK = f_ext/virtual_damper_DK_z - virtual_spring_DK_z/virtual_damper_DK_z * Z_position_from_model_DK;
  Z_position_from_model_DK = Z_position_from_model_DK + Z_position_dot_from_model_DK * time_loop;

  Z_cmd = ref - Z_position_from_model_DK;
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
  KDL::JntArray q(kdl_chain_.getNrOfJoints());

  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) 
  {
    q(i) = measured_angle[i];
  }

  KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
  KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
  jac_solver.JntToJac(q, jacobian);

  // Eigen 행렬로 변환
  Eigen::MatrixXd J(6, kdl_chain_.getNrOfJoints());
  for (int i = 0; i < 6; ++i) 
  {
    for (int j = 0; j < kdl_chain_.getNrOfJoints(); ++j) 
    {
        J(i, j) = jacobian(i, j);
    }
  }

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