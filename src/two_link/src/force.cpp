#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "open_manipulator_msgs/KinematicsPose.h"
#include <iostream>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 

#define PI 3.141592

double effort1 = 0;
double effort2 = 0;
double effort3 = 0;
double effort4 = 0;
double velocity1_i = 0;
double velocity1_f = 0;
double velocity2_i = 0;
double velocity2_f = 0;
double velocity3_i = 0;
double velocity3_f = 0;
double velocity4_i = 0;
double velocity4_f = 0;
double d_velocity1 = 0;
double d_velocity2 = 0;
double d_velocity3 = 0;
double d_velocity4 = 0;
double x = 0;
double y = 0;
double result = 0;
double divide = 0;
double t_i = 0;
double t_f = 0;
double dt = 0;
double w_i = 0;
double w_f = 0;
double alpha1_i = 0;
double alpha1_f = 0;
double alpha2_i = 0;
double alpha2_f = 0;
double alpha3_i = 0;
double alpha3_f = 0;
double alpha4_i = 0;
double alpha4_f = 0;
double alpha1_LPF_i = 0;
double alpha1_LPF_f = 0;
double alpha2_LPF_i = 0;
double alpha2_LPF_f = 0;
double alpha3_LPF_i = 0;
double alpha3_LPF_f = 0;
double alpha4_LPF_i = 0;
double alpha4_LPF_f = 0;
double effort1_LPF_i = 0;
double effort2_LPF_i = 0;
double effort3_LPF_i = 0;
double effort4_LPF_i = 0;
double effort1_LPF_f = 0;
double effort2_LPF_f = 0;
double effort3_LPF_f = 0;
double effort4_LPF_f = 0;
double F_Endeffector_X;
double F_Endeffector_Y;
double F_Endeffector_Z;

geometry_msgs::Twist test;
geometry_msgs::Twist force;

//double Cut_Off_Frequency = 100;
double WL = 0.3;//2 * 3.141592 / Cut_Off_Frequency;


//---------------------[Set Parameters]----------------------//

// mass, width, height of link[MKS]
double m2 = 0.147;
double m3 = 0.122;
double m4 = 0.107;

double a2 = 0.12774;
double a3 = 0.12409;
double a4 = 0.14909;

double h2 = 0.021;
double h3 = 0.021;
double h4 = 0.021;

double lc2 = 0.10373; //0.10194;
double lc3 = 0.09317; //0.9273;
double lc4 = 0.05772;//0.0041;

double g = -9.81; // Gravitational Acceleration

double Kt = 0.0044; // mA 단위            
double Kt_2 = 0.0074; // mA 단위  보정값
double Kt_3 = 0.00030;// 0.0004187934426; // mA 단위
double Kt_4 = 0.0044; // mA 단위

double K_G1 = 2.5;
double K_G2 = 11;
double K_G3 = 1;



double max_1 = 0.5;
double min_1 = -0.5;
double max_2 = +0.5;    //2번조인트
double min_2 = -0.5;
double max_3 = 0.6;     //3번조인트
double min_3 = -0.6;
double max_4 = 0.05;
double min_4 = -0.05;
double G_Tau1 = 0;
double G_Tau2 = 0;
double G_Tau3 = 0;
double Tau_d1 = 0;
double Tau_d2 = 2.8;
double Tau_d3 = -0.02;

Eigen::Vector3d G_Tau;



//double Cut_Off_Frequency = 100;
//double WL = 0.3;//2 * 3.141592 / Cut_Off_Frequency;

//Momentum Of Inertia
double I2 = m2 * (pow(h2,2) + pow(a2,2)) / 12;
double I3 = m3 * (pow(h3,2) + pow(a3,2)) / 12;
double I4 = m4 * (pow(h4,2) + pow(a4,2)) / 12;

//---------------------[Set Parameters]----------------------//
// angle[rad]
double angle1;
double angle2;
double angle3;
double angle4;

double q1;
double q2;
double q3;
double q4;

double delta2 = 0.067;
double e2 = 0.02495;

//---------------------[Matrix Variation]----------------------//
// Inertia Matrix Elements
double M11;
double M12;
double M13;
double M21;
double M22;
double M23;
double M31;
double M32; 
double M33;

// Centrifugual and Coriolis Matrix Elements(except angular_vel)
double A112;
double A123;
double A113;
double A122;
double A133;
double A211;
double A233;
double A212;
double A223;
double A213;
double A311;
double A322;
double A312;
double A323;
double A313;

// Centrifugual and Coriolis Matrix Elements(include angular_vel)
double C1;
double C2;
double C3;

// Gravity Matrix Elements
double G1;
double G2;
double G3;

// Jacobian Elements
double J11;
double J12;
double J13;
double J21;
double J22;
double J23;

//---------------------[Matrix Variation]----------------------//
// Fx, Fy, Fz(External Force)
double Fx;
double Fy;
double Fz;

//Torque of joint 1, 2, 3 and 4
double tau1;
double tau2;
double tau3;
double tau4;

// Define Matrix
Eigen::Matrix3d M;    // Inertia
Eigen::Vector3d C;    // Centrifugual and Coriolis
Eigen::Vector3d G;    // Gravity
Eigen::MatrixXd J(2, 3);    // Jacobian
Eigen::MatrixXd JT;    // Transpose
Eigen::MatrixXd JTI;    // Transpose+Inverse
Eigen::Vector2d F;    // External Force(Fx, Fz)
Eigen::Vector2d F_T;
Eigen::Vector3d T;    // Torque of Joint 2,3 and4
Eigen::Vector3d Ang_acc;   // Angular acceleration
Eigen::Matrix3d RotZ_Inverse;   // RotationZ Inverse matrix
Eigen::Vector3d F_World;    // External Force(Fx, Fy, Fz)
Eigen::Vector3d F_World_T;    // External Force(Fx, Fy, Fz)
Eigen::Vector3d F_Endeffector;    // External Force(Rotation -> Endeffector)
Eigen::Vector3d F_Endeffector_T; // 
Eigen::Vector3d Tau_d; // 보정




template<typename Derived>
Derived dampedPinv(const Eigen::MatrixBase<Derived>& a, double rho = 1e-4) 
{
	return a.transpose() * (a * a.transpose() + rho * rho * Eigen::MatrixBase<Derived>::Identity(a.rows(), a.rows())).inverse();
}
//---------------------[Matrix Variation]----------------------//




//---------------------[Functions]----------------------//
void EffortToTorque(const sensor_msgs::JointState::ConstPtr& msg);
void XYCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr& msg);
void setMomentMatrix();
void setColiolisMatrix();
void setGravityMatrix();
void setJacobianMatrix();
void EstimateForce();
void RotZForce();

//---------------------[Functions]----------------------//



int main(int argc, char** argv)
{
    ros::init(argc, argv, "force");
    ros::NodeHandle n;

    ros::Publisher testerpub = n.advertise<geometry_msgs::Twist>("/tester", 10);
    ros::Publisher forcepub = n.advertise<geometry_msgs::Twist>("/estimated_force",10);

    ros::Subscriber effortsub = n.subscribe("/joint_states", 1000, EffortToTorque);  //각도(변수 다름), 각속도(변수 다름), 각가속도, LPF 4개를 받아오는 거
    ros::Subscriber xysub = n.subscribe("/gripper/kinematics_pose", 1000, XYCallback);




    ros::Rate loop(50);
    while (ros::ok())
    {
        //EffortToTorque(const sensor_msgs::JointState::ConstPtr& msg);
        setMomentMatrix();
        setColiolisMatrix();
        setGravityMatrix();
        setJacobianMatrix();
        EstimateForce();
        RotZForce();
        //std::cout<<std::endl<<std::endl<<std::endl;
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}




void EffortToTorque(const sensor_msgs::JointState::ConstPtr& msg)
{
    ros::NodeHandle n;
    ros::Publisher testerpub = n.advertise<geometry_msgs::Twist>("/tester", 10);

    t_f = ros::Time::now().toSec();

    angle1 = msg->position.at(0); // angle
    angle2 = msg->position.at(1); // angle
    angle3 = msg->position.at(2); // angle
    angle4 = msg->position.at(3); // angle

    q1 = -angle1;
    q2 = -angle2 + PI / 2;
    q3 = -angle3 - PI / 2;
    q4 = -angle4; // 각도 변경

    ROS_INFO("q1 : %lf, q2 : %lf, q3 : %lf, q4 : %lf", q1, q2, q3, q4);

    effort1 = msg->effort.at(0); //effort
    effort2 = msg->effort.at(1);
    effort3 = msg->effort.at(2);
    effort4 = msg->effort.at(3);

    velocity1_f = msg->velocity.at(0); //angular velocity
    velocity2_f = msg->velocity.at(1);
    velocity3_f = msg->velocity.at(2);
    velocity4_f = msg->velocity.at(3);

    tau1 = (effort1_LPF_f) * Kt; // torque
    tau2 = (effort2_LPF_f) * Kt; // torque
    tau3 = (effort3_LPF_f) * Kt; // torque
    tau4 = (effort4_LPF_f) * Kt; // torque

//----------------[angle, velocity, torque initialize]----------------//

    dt = t_f - t_i;
    d_velocity1 = velocity1_f - velocity1_i;
    d_velocity2 = velocity2_f - velocity2_i;
    d_velocity3 = velocity3_f - velocity3_i;
    d_velocity4 = velocity4_f - velocity4_i;

    alpha1_f = d_velocity1 / dt;  //alpha : 수치미분해서 구한 각가속도
    alpha2_f = d_velocity2 / dt;  //alpha : 수치미분해서 구한 각가속도
    alpha3_f = d_velocity3 / dt;  //alpha : 수치미분해서 구한 각가속도
    alpha4_f = d_velocity4 / dt;  //alpha : 수치미분해서 구한 각가속도

//----------------[(i+1) - (i)]------------------/


    alpha1_LPF_f = WL * alpha1_f + (1 - WL) * alpha1_LPF_i;
    alpha2_LPF_f = WL * alpha2_f + (1 - WL) * alpha2_LPF_i;
    alpha3_LPF_f = WL * alpha3_f + (1 - WL) * alpha3_LPF_i;
    alpha4_LPF_f = WL * alpha4_f + (1 - WL) * alpha4_LPF_i;
    effort1_LPF_f = WL* effort1 + (1 - WL) * effort1_LPF_i;
    effort2_LPF_f = WL* effort2 + (1 - WL) * effort2_LPF_i;
    effort3_LPF_f = WL* effort3 + (1 - WL) * effort3_LPF_i;
    effort4_LPF_f = WL* effort4 + (1 - WL) * effort4_LPF_i;


    test.linear.x = F_Endeffector[0]; //tau2 - Tau_d[0];//G_Tau[0]; //law data
    test.linear.y = F_Endeffector[1]; //tau3 - Tau_d[1];//tau3 - Tau_d[1];//G_Tau[1]; // 교수님이 알려주신 버전   이때 WL = 2*pi / CutOff Frequency    즉, Cut Off Frequency = 2*pi / WL
    test.linear.z = F_Endeffector[2]; //tau4 - Tau_d[2];//G_Tau[2]; // 원래 쓰던거
    test.angular.x = F_Endeffector_T[0];
    test.angular.y = F_Endeffector_T[1];
    test.angular.z = F_Endeffector_T[2];


//----------------[LOW PASS FILTER]------------------/


//----------------[about i]------------------/

    t_i = ros::Time::now().toSec();

    velocity1_i = velocity1_f;
    velocity2_i = velocity2_f;
    velocity3_i = velocity3_f;
    velocity4_i = velocity4_f;

    alpha1_i = alpha1_f;
    alpha2_i = alpha2_f;
    alpha3_i = alpha3_f;
    alpha4_i = alpha4_f;

    alpha1_LPF_i = alpha1_LPF_f;
    alpha2_LPF_i = alpha2_LPF_f;
    alpha3_LPF_i = alpha3_LPF_f;
    alpha4_LPF_i = alpha4_LPF_f;
    effort1_LPF_i = effort1_LPF_f;
    effort2_LPF_i = effort2_LPF_f;
    effort3_LPF_i = effort3_LPF_f;
    effort4_LPF_i = effort4_LPF_f;

    testerpub.publish(test);

//----------------[angle, velocity, torque initialize]----------------//

    T << tau2, tau3, tau4;
    //ROS_INFO("tau2 : %lf, tau3 : %lf, tau4 : %lf", tau2, tau3, tau4);
}

void XYCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr& msg)
{
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    //ROS_INFO("%lf, %lf", x, y);
    result = sqrt(pow(x, 2) + pow(y, 2));
    //ROS_INFO("result = %lf",result);

    ///////////////////////////////////////////////////////////////////////////////////
    //tau1 = result * Fy;
    //ROS_INFO("tau1 = %lf",tau1);
    Fy = tau1 / result;
    F_World[1] = Fy;
    F_World_T[1] = Fy;
    //ROS_WARN("Fy = %lf", Fy);
}

void setMomentMatrix()
{
    M11 = m2 * pow(lc2,2) + I2 + m3 * pow(a2,2) + m3 * pow(lc3,2) + 2 * m3 * a2 * lc3 * cos(q3) + I3 + m4 * pow(a2,2) + m4 * pow(a3,2) + m4 * pow(lc4,2) + m4 * a2 * a3 * 2 * cos(q3) + m4 * a3 * lc4 * 2 * cos(q4) + m4 * a2 * lc4 * 2 * cos(q3 + q4) + I4; // modified
    M12 = m3 * pow(lc3,2) + m3 * a2 * lc3 * cos(q3) + I3 + m4 * pow(a3,2) + m4 * pow(lc4,2) + m4 * a2 * a3 * cos(q3) + m4 * a3 * lc4 * 2 * cos(q4) + m4 * a2 * lc4 * cos(q3 + q4) + I4; // modified
    M13 = m4 * pow(lc4,2) + m4 * a3 * lc4 * cos(q4) + m4 * a2 * lc4 * cos(q3 + q4) + I4;
    M21 = m3 * pow(lc3,2) + I4 + m3 * a2 * lc3 * cos(q3) + I3 + m4 * pow(a3,2) + m4 * pow(lc4,2) + m4 * a2 * a3 * cos(q3) + 2 * m4 * a3 * lc4 * cos(q4) + m4 * a2 * lc4 * cos(q3 + q4);/////////////////////////////////////////////
    M22 = m3 * pow(lc3,2) + I4 + I3 + m4 * pow(a3,2) + m4 * pow(lc4,2) + 2 * m4 * a3 * lc4 * cos(q4);
    M23 = m4 * pow(lc4,2) + m4 * a3 * lc4 * cos(q4) + I4;
    M31 = m4 * pow(lc4,2) + m4 * a3 * lc4 * cos(q4) + m4 * a2 * lc4 * cos(q3 + q4) + I4;
    M32 = m4 * pow(lc4,2) + m4 * a3 * lc4 * cos(q4) + I4;
    M33 = m4 * pow(lc4,2) + I4;

    M << M11, M12, M13,
         M21, M22, M23,
         M31, M32, M33;

    Ang_acc << alpha2_LPF_f, alpha3_LPF_f, alpha4_LPF_f;
    //ROS_INFO("M11: %lf, M12: %lf, M13: %lf, M21: %lf, M22: %lf, M23: %lf, M31: %lf, M32: %lf, M33: %lf", M11, M12, M13, M21, M22, M23, M31, M32, M33);
}

void setColiolisMatrix()
{
    // Centrifugual and Coriolis Matrix Elements(except angular_vel)
    A112 = -m3 * a2 * lc3 * 2 * sin(q3) - m4 * a2 * a3 * 2 * sin(q3) - 2 * m4 * a2 * lc4 * sin(q3 + q4);
    A123 = -m4 * a3 * lc4 * 2 * sin(q4) - 2 * m4 * a2 * lc4 * sin(q3 + q4);
    A113 = -2 * m4 * a3 * lc4 * sin(q4) - 2 * m4 * a2 * lc4 * sin(q3 + q4);
    A122 = -m3 * a2 * lc4 * sin(q4) - m4 * a2 * a3 * sin(q3) - m4 * a2 * lc4 * sin(q3 + q4);
    A133 = -m4 * a2 * lc4 * sin(q3 + q4) - m4 * a3 * lc4 * sin(q4);
    A211 = m3 * a2 * lc3 * sin(q3) + m4 * a2 * a3 * sin(q3) + m4 * a2 * lc4 * sin(q3 + q4); // modified
    A233 = -m4 * a3 * lc4 * sin(q4);
    A212 = 0;
    A223 = -m4 * a3 * lc4 * 2 * sin(q4);
    A213 = -2 * m4 * a3 * lc4 * sin(q4);
    A311 = m4 * a3 * lc4 * sin(q4) + m4 * a2 * lc4 * sin(q3 + q4);
    A322 = m4 * a3 * lc4 * sin(q4);
    A312 = 2 * m4 * a3 * lc4 * sin(q4);
    A323 = 0;
    A313 = 0;

    // Centrifugual and Coriolis Matrix Elements(include angular_vel)
    C1 = A112 * velocity2_f * velocity3_f + A123 * velocity3_f * velocity4_f + A113 * velocity2_f * velocity4_f + A133 * pow(velocity4_f,2) + A122 * pow(velocity3_f,2);
    C2 = A211 * pow(velocity2_f,2) + A233 * pow(velocity4_f,2) + A212 * velocity2_f * velocity3_f + A223 * velocity3_f * velocity4_f + A213 * velocity2_f * velocity4_f;
    C3 = A311 * pow(velocity2_f,2) + A322 * pow(velocity3_f,2) + A312 * velocity2_f * velocity3_f + A323 * velocity3_f * velocity4_f + A313 * velocity2_f * velocity4_f;

    C << C1, C2, C3;
    //ROS_INFO("C1: %lf, C2: %lf, C3: %lf", C1, C2, C3);
}

void setGravityMatrix()
{
    // Gravity Matrix Elements
    /* G1 = m2 * g * lc2 * cos(q2) + m3 * g * (a2 * cos(q2) + lc3 * cos(q2 + q3)) + m4 * g * (a2 * cos(q2) + a3 * cos(q2 + q3) + lc4 * cos(q2 + q3 + q4));
    G2 = m3 * g * lc3 * cos(q2 + q3) + m4 * g * (a3 * cos(q2 + q3) + lc4 * cos(q2 + q3 + q4));
    G3 = m4 * g * lc4 * cos(q2 + q3 + q4); */
    G1 = (lc2 * m2 * g * cos(q2 - delta2) + m3 * g * (a2 * cos(q2) + e2 *sin(q2) + lc3 * cos(q2 + q3)) + m4 * g * (a2 * cos(q2) + e2 * sin(q1) + a3 * cos(q2 + q3) + lc4 *cos(q2 + q3 + q4))) * K_G1;
    G2 = -3.2;//(m3 * g * lc3 * cos(q2 + q3) + m4 * g * (a3 * cos(q2 + q3) + lc4 * cos(q2 + q3 + q4))) * K_G2;
    G3 = -0.05;//(m4 * g * lc4 * cos(q2 + q3 + q4)) * K_G3;

    G << G1, G2, G3;

    G_Tau1 = G1 - tau2 + Tau_d1;
    G_Tau2 = G2 - tau3 + Tau_d2;
    G_Tau3 = G3 - tau4 + Tau_d3;


   if(tau1 <= max_1 && tau1 >= min_1) tau1 = 0;
   else if(tau1 > max_1) tau1 = tau1 - max_1;
   else if(tau1 < min_1) tau1 = tau1 - min_1;

   if(G_Tau1 <= max_2 && G_Tau1 >= min_2) G_Tau1 = 0;
   else if(G_Tau1 > max_2) G_Tau1 = G_Tau1 - max_2;
   else if(G_Tau1 < min_2) G_Tau1 = G_Tau1 - min_2;

   if(G_Tau2 <= max_3 && G_Tau2 >= min_3) G_Tau2 = 0;
   else if(G_Tau2 > max_3) G_Tau2 = G_Tau2 - max_3;
   else if(G_Tau2 < min_3) G_Tau2 = G_Tau2 - min_3;

   if(G_Tau3 <= max_4 && G_Tau3 >= min_4) G_Tau3 = 0;
   else if(G_Tau3 > max_4) G_Tau3 = G_Tau3 - max_4;
   else if(G_Tau3 < min_4) G_Tau3 = G_Tau3 - min_4;



    G_Tau << G_Tau1, G_Tau2, G_Tau3;

    //ROS_INFO("G1: %lf, G2: %lf, G3: %lf", G1, G2, G3);
}

void setJacobianMatrix()
{
    J11 = -(a2 * sin(q2) + a3 * sin(q2 + q3) + a4 * sin(q2 + q3 + q4)) + e2 * cos(q2);
    J12 = -(a3 * sin(q2 + q3) + a4 * sin(q2 + q3 + q4));
    J13 = -a4 * sin(q2 + q3 + q4);
    J21 = a2 * cos(q2) + a3 * cos(q2 + q3) + a4 * cos(q2 + q3 + q4) + e2 * sin(q2);
    J22 = a3 * cos(q2 + q3) + a4 * cos(q2 + q3 + q4);
    J23 = a4 * cos(q2 + q3 + q4);
        
    J << J11, J12, J13, J21, J22, J23;

    JT = J.transpose();
    JTI = dampedPinv(JT); //pseudo inverse
    //ROS_INFO("J11: %lf, J12: %lf, J13: %lf, J21: %lf, J22: %lf, J23: %lf", J11, J12, J13, J21, J22, J23);
    std::cout<<JTI<<std::endl;
}

void EstimateForce()
{
    Tau_d << Tau_d1, Tau_d2, Tau_d3;
    F = JTI * (G_Tau);//F = JTI * (G - T + Tau_d); //
    F_T = JTI * (-T + Tau_d);
    Fx = F[0];
    Fz = F[1];
    F_World[0] = F[0];
    F_World[2] = F[1];
    F_World_T[0] = F_T[0];
    F_World_T[2] = F_T[1];
    //ROS_INFO("Fx : %lf, Fz : %lf", Fx, Fz);
    //ROS_WARN("tau4*G2 : %lf, tau3*G3 : %lf", tau4*G2, tau3*G3);
    //ROS_WARN("tau3*G1 : %lf, tau2*G2 : %lf", tau3*G1, tau2*G2);
}

void RotZForce()
{
    ros::NodeHandle n;
    ros::Publisher forcepub = n.advertise<geometry_msgs::Twist>("/estimated_force", 10);

    RotZ_Inverse << cos(angle1), sin(angle1), 0,
                   -sin(angle1), cos(angle1), 0,
                         0,          0,       1;
    
    F_Endeffector = RotZ_Inverse * F_World;
    F_Endeffector_T = RotZ_Inverse * F_World_T;

    ROS_WARN("World_Fx : %lf, World_Fy : %lf, World_Fz : %lf", F_World[0], F_World[1], F_World[2]);
    //ROS_INFO("Rot_Fx : %lf, Rot_Fy : %lf, Rot_Fz : %lf", F_Endeffector[0], F_Endeffector[1], F_Endeffector[2]);
    
    force.linear.x = F_Endeffector_T[0];
    force.linear.y = F_Endeffector_T[1];
    force.linear.z = F_Endeffector_T[2];
    force.angular.x = F_Endeffector[0];
    force.angular.y = F_Endeffector[1];
    force.angular.z = F_Endeffector[2];

    forcepub.publish(force);
}