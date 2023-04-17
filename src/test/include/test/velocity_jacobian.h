#ifndef VELOCITY_JACOBIAN_H_
#define VELOCITY_JACOBIAN_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <dynamixel_workbench_msgs/DasomDynamixel.h>

#define PI 3.14159256359

class VelJ
{
 public:
  VelJ();
  ~VelJ();

  double p_gain;
  double i_gain;
  double d_gain;
  double command_x_position_fromGUI = 0;
  double command_y_position_fromGUI = 0;
  double command_z_position_fromGUI = 0;
  double command_r_angle_fromGUI = 0;
  double command_p_angle_fromGUI = 0;
  double command_yaw_angle_fromGUI = 0;
  double position_x_dot = 0;
  double position_y_dot = 0;
  double position_z_dot = 0;
  double angle_r_dot = 0;
  double angle_p_dot = 0;
  double angle_yaw_dot = 0;
  double measured_x_position = 0; // 현재 x값(위치)
  double measured_y_position = 0;
  double measured_z_position = 0;
  double measured_r_angle = 0;
  double measured_p_angle = 0;
  double measured_yaw_angle = 0;
  double measured_velocity = 0;
  double l1 = 0.04233;
  double l2 = 0.12409;
  double l3 = 0.04794;
  double l4 = 0.06133;
  double l5 = 0.04583;
  double l6 = 0.06733;
  double l7 = 0.1;

  //Eigen::MatrixXd X_dot;
  Eigen::VectorXd X_dot;
  Eigen::MatrixXd JT;
  Eigen::Vector3d EE_position;
  Eigen::Vector3d EE_orient;
  //Eigen::MatrixXd q_dot;
  Eigen::VectorXd q_dot;

  void linearvelocityJ();
  void calc_qdot();

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  std::string robot_name_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Publisher dasom_command_velocity_pub_;
  ros::Subscriber dasom_joint_states_sub;
  ros::Subscriber dasom_command_sub;

  static Eigen::MatrixXd aX_dot(double x,double y,double z,double r,double p,double yaw)
{
    Eigen::MatrixXd xdot(1,6);

    xdot << x, y, z, r, p, yaw;

    return xdot;
}

  static Eigen::Matrix4d L1(double theta_1)
{
    double l1 = 0.04233;
    double cosT = cos(theta_1), sinT = sin(theta_1);

    Eigen::Matrix4d L;
    L <<      cosT,     -sinT,     0,       0,
              sinT,      cosT,     0,       0,
                 0,         0,     1,      l1,
                 0,         0,     0,       1;
    return L;
};

  static Eigen::Matrix4d L2(double theta_2)
{
    double l2 = 0.12409;
    double cosT = cos(theta_2), sinT = sin(theta_2);

    Eigen::Matrix4d L;
    L <<         1,         0,     0,       0,
                 0,      cosT, -sinT, l2*cosT,
                 0,      sinT,  cosT, l2*sinT,
                 0,         0,     0,       1;
    return L;
};

  static Eigen::Matrix4d L3(double theta_3)
{
    double l3 = 0.04794;
    double cosT = cos(theta_3), sinT = sin(theta_3);

    Eigen::Matrix4d L;
    L <<         1,         0,     0,       0,
                 0,      cosT, -sinT, l3*cosT,
                 0,      sinT,  cosT, l3*sinT,
                 0,         0,     0,       1;
    return L;
};

  static Eigen::Matrix4d L4(double theta_4)
{
    double l4 = 0.06133;
    double cosT = cos(theta_4), sinT = sin(theta_4);

    Eigen::Matrix4d L;
    L <<      cosT,         0,  sinT, l4*sinT,
                 0,         1,     0,       0,
             -sinT,         0,  cosT, l4*cosT,
                 0,         0,     0,       1;
    return L;
};

  static Eigen::Matrix4d L5()
{
    double l5 = 0.04583;
    Eigen::Matrix4d L;
    L <<         1,         0,     0,       0,
                 0,         1,     0,      l5,
                 0,         0,     1,       0,
                 0,         0,     0,       1;
    return L;
};

  static Eigen::Matrix4d L6(double theta_5)
{
    double l6 = 0.06733;
    double cosT = cos(theta_5), sinT = sin(theta_5);

    Eigen::Matrix4d L;
    L <<      cosT,     -sinT,     0,       0,
              sinT,      cosT,     0,       0,
                 0,         0,     1,     -l6,
                 0,         0,     0,       1;
    return L;
};

  static Eigen::Matrix4d L7(double theta_6)
{
    double l7 = 0.1;
    double cosT = cos(theta_6), sinT = sin(theta_6);

    Eigen::Matrix4d L;
    L <<         1,         0,     0,       0,
                 0,      cosT, -sinT, l7*cosT,
                 0,      sinT,  cosT, l7*sinT,
                 0,         0,     0,       1;
    return L;
};

static Eigen::MatrixXd EE_pos(double theta_1,double theta_2,double theta_3,
			      double theta_4,double theta_5,double theta_6)
{
    double l1 = 0.04233;
    double l2 = 0.12409;
    double l3 = 0.04794;
    double l4 = 0.06133;
    double l5 = 0.04583;
    double l6 = 0.06733;
    double l7 = 0.1;
    double cos1 = cos(theta_1), sin1 = sin(theta_1);
    double cos2 = cos(theta_2), sin2 = sin(theta_2);
    double cos3 = cos(theta_3), sin3 = sin(theta_3);
    double cos4 = cos(theta_4), sin4 = sin(theta_4);
    double cos5 = cos(theta_5), sin5 = sin(theta_5);
    double cos6 = cos(theta_6), sin6 = sin(theta_6);
    double sin23 = sin(theta_2 + theta_3);
    double cos23 = cos(theta_2 + theta_3);
    double cos45 = cos(theta_4 + theta_5);
    double sin46 = sin(theta_4 + theta_6);
    double cos4m5 = cos(theta_4 - theta_5);
    double sin4m6 = sin(theta_4 - theta_6);

    Eigen::MatrixXd EE_pos(3,1);

    EE_pos << 
// X
l5*(sin1*sin2*sin3 - cos2*cos3*sin1) - l6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) + l4*cos4*(cos2*sin1*sin3 + cos3*sin1*sin2) - l7*cos6*(sin5*(cos1*cos4 - sin4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - cos5*(sin1*sin2*sin3 - cos2*cos3*sin1)) - l2*cos2*sin1 + l4*cos1*sin4 + l7*sin6*(cos1*sin4 + cos4*(cos2*sin1*sin3 + cos3*sin1*sin2)) - l3*cos2*cos3*sin1 + l3*sin1*sin2*sin3,
// Y
l2*cos1*cos2 - l6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l5*(cos1*sin2*sin3 - cos1*cos2*cos3) + l4*sin1*sin4 + l7*sin6*(sin1*sin4 - cos4*(cos1*cos2*sin3 + cos1*cos3*sin2)) - l7*cos6*(sin5*(cos4*sin1 + sin4*(cos1*cos2*sin3 + cos1*cos3*sin2)) + cos5*(cos1*sin2*sin3 - cos1*cos2*cos3)) - l4*cos4*(cos1*cos2*sin3 + cos1*cos3*sin2) + l3*cos1*cos2*cos3 - l3*cos1*sin2*sin3,
// Z
l1 + l3*sin23 + l5*sin23 + l2*sin2 + (l7*cos23*sin46)/2 + l4*cos23*cos4 - l6*cos23*cos4 - (l7*sin4m6*cos23)/2 + l7*cos6*((cos4m5*cos23)/2 - (cos23*cos45)/2 + sin23*cos5);

    return EE_pos;

}

static Eigen::MatrixXd EE_orientation(double theta_1,double theta_2,double theta_3,
			      double theta_4,double theta_5,double theta_6)
{
    double r11 = cos(theta_5)*(cos(theta_1)*cos(theta_4) - sin(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) + sin(theta_5)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_2)*cos(theta_3)*sin(theta_1));
    double r12 = sin(theta_6)*(cos(theta_1)*sin(theta_4) + cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) - cos(theta_6)*(sin(theta_5)*(cos(theta_1)*cos(theta_4) - sin(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) - cos(theta_5)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_2)*cos(theta_3)*sin(theta_1)));
    double r13 = cos(theta_6)*(cos(theta_1)*sin(theta_4) + cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) + sin(theta_6)*(sin(theta_5)*(cos(theta_1)*cos(theta_4) - sin(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2))) - cos(theta_5)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_2)*cos(theta_3)*sin(theta_1)));
    double r21 = cos(theta_5)*(cos(theta_4)*sin(theta_1) + sin(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) - sin(theta_5)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3));
    double r22 = sin(theta_6)*(sin(theta_1)*sin(theta_4) - cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) - cos(theta_6)*(sin(theta_5)*(cos(theta_4)*sin(theta_1) + sin(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) + cos(theta_5)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3)));
    double r23 = cos(theta_6)*(sin(theta_1)*sin(theta_4) - cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) + sin(theta_6)*(sin(theta_5)*(cos(theta_4)*sin(theta_1) + sin(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2))) + cos(theta_5)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3)));
    double r31 = sin(theta_2 + theta_3)*sin(theta_5) - cos(theta_2 + theta_3)*cos(theta_5)*sin(theta_4);
    double r32 = cos(theta_6)*(sin(theta_2 + theta_3)*cos(theta_5) + cos(theta_2 + theta_3)*sin(theta_4)*sin(theta_5)) + cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_6);
    double r33 = cos(theta_2 + theta_3)*cos(theta_4)*cos(theta_6) - sin(theta_6)*(sin(theta_2 + theta_3)*cos(theta_5) + cos(theta_2 + theta_3)*sin(theta_4)*sin(theta_5));

//    double beta = asin2(-r31);
//
//    Eigen::MatrixXd EE_Orientation(3,1);
//    
//    EE_Orientation << 
//  // roll
//    atan2(r32/cos(beta),r33/cos(beta)),
//    // pitch
//    beta,
//    // yaw
//    atan2(r21/cos(beta),r11/cos(beta));

    double alpha = atan2(r21,r11);

    Eigen::MatrixXd EE_Orientation(3,1);
    
    EE_Orientation << 
    // roll
    atan2(sin(alpha)*r13-cos(alpha)*r23,-sin(alpha)*r12+cos(alpha)*r22),
    // pitch
    atan2(-r31,cos(alpha)*r11+sin(alpha)*r21),				// limit pitch direction
    // yaw
    alpha;


    return EE_Orientation;

}

static Eigen::MatrixXd Jacobian(double theta_1,double theta_2,double theta_3,
			        double theta_4,double theta_5,double theta_6)
{
    double l1 = 0.04233;
    double l2 = 0.12409;
    double l3 = 0.04794;
    double l4 = 0.06133;
    double l5 = 0.04583;
    double l6 = 0.06733;
    double l7 = 0.1;
    double cos1 = cos(theta_1), sin1 = sin(theta_1);
    double cos2 = cos(theta_2), sin2 = sin(theta_2);
    double cos3 = cos(theta_3), sin3 = sin(theta_3);
    double cos4 = cos(theta_4), sin4 = sin(theta_4);
    double cos5 = cos(theta_5), sin5 = sin(theta_5);
    double cos6 = cos(theta_6), sin6 = sin(theta_6);
    double sin23 = sin(theta_2 + theta_3);
    double cos23 = cos(theta_2 + theta_3);
    double cos45 = cos(theta_4 + theta_5);
    double sin46 = sin(theta_4 + theta_6);
    double cos4m5 = cos(theta_4 - theta_5);
    double sin4m6 = sin(theta_4 - theta_6);

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



  void poseCallback(const dynamixel_workbench_msgs::DasomDynamixel::ConstPtr &msg);
  void commandCallback(const geometry_msgs::Twist &msg);

  //void goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg);

};

#endif //VelJ_H_
