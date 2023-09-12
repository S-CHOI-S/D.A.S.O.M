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

#include "ros/ros.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
 
using namespace KDL;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "try");
    ros::NodeHandle nh;
    ROS_WARN("try node start!");

    ros::Rate loop_rate(200);

    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
    // 0.0
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.05465)),RigidBodyInertia(0.5,Vector(0,0,0.027325),RotationalInertia(0,0,0,0,0,0))));
    // 0.0 0.263367
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.1549,0.0)),RigidBodyInertia(0.22226,Vector(0.00039,-0.03411,0.0),RotationalInertia(0,0,0,0,0,0))));
    // 0.0 0.577486	0.117183	
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.116,0.0)),RigidBodyInertia(0.1296,Vector(-0.00027,-0.02383,-0.00886),RotationalInertia(0,0,0,0,0,0))));
    // 0.0 0.641148	0.146350 0.000080
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.05075,-0.06183)),RigidBodyInertia(0.0227,Vector(-0.00036,-0.03577,0.02293),RotationalInertia(0,0,0,0,0,0))));
    // 0.0 0.991254	0.329760 -0.046625 0.000000	
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.0613)),RigidBodyInertia(0.1097,Vector(0.0434,0.00368,-0.02326),RotationalInertia(0,0,0,0,0,0))));
    // 0.0 2.110560	0.986372 0.085612 0.000000 0.158523	
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.115,0)),RigidBodyInertia(0.30449,Vector(-0.04427,-0.06193,-0.00627),RotationalInertia(0,0,0,0,0,0))));
 
    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);

    double i = 0;

    ros::Publisher joint_pub = nh.advertise<geometry_msgs::Twist>("/joint1", 100);
    geometry_msgs::Twist test;
 
    // Assign some values to the joint positions
    // for(unsigned int i=0;i<nj;i++){
    //     float myinput;
        // printf ("Enter the position of joint %i: ",i);
        // scanf ("%e",&myinput);
        // jointpositions(i)=(double)myinput;
    // }

    while(ros::ok())
    {
        jointpositions(0) = 0;
        jointpositions(1) = 3.141592/2;//-1.75*cos(i/1000)+1.75;
        jointpositions(2) = 0;
        jointpositions(3) = 0;
        jointpositions(4) = 0;
        jointpositions(5) = 3.141592/2;
        
        // ROS_INFO("joint2 = %lf",jointpositions(1));
    
        // Create the frame that will contain the results
        KDL::Frame cartpos;    
    
        // Calculate forward position kinematics
        bool kinematics_status;
        kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
        // if(kinematics_status>=0){
            std::cout << cartpos <<std::endl;
            // printf("%s \n","Succes, thanks KDL!");
        // }else{
        //     printf("%s \n","Error: could not calculate forward kinematics :(");
        // }

        KDL::JntArray q(chain.getNrOfJoints());
        KDL::JntArray qdot(chain.getNrOfJoints());
        KDL::JntArray qdotdot(chain.getNrOfJoints());
        KDL::JntArray tau(chain.getNrOfJoints());
        KDL::JntArray tauHCGa(chain.getNrOfJoints());
        KDL::JntArray tauHCG(chain.getNrOfJoints());
        KDL::JntArray C(chain.getNrOfJoints()); //coriolis matrix
        KDL::JntArray G(chain.getNrOfJoints()); //gravity matrix
        KDL::Wrenches f(chain.getNrOfSegments());
        KDL::Vector grav(0.0,0.0,-9.81);
        KDL::JntSpaceInertiaMatrix H(chain.getNrOfJoints()); //inertiamatrix H=square matrix of size= number of joints
        KDL::ChainDynParam chaindynparams(chain,grav);  

        qdot(0) = 0;
        qdot(1) = 0;//1.75*0.001*sin(i/1000);
        qdot(2) = 0;
        qdot(3) = 0;
        qdot(4) = 0;
        qdot(5) = 0;

        //calculation of the HCG matrices
        chaindynparams.JntToMass(jointpositions,H);
        chaindynparams.JntToCoriolis(jointpositions,qdot,C);
        chaindynparams.JntToGravity(jointpositions,G);

        // Jacobian 계산
        KDL::Jacobian jacobian;  // Jacobian 행렬
        KDL::ChainJntToJacSolver jac_solver(chain); // Jacobian 계산용 Solver 생성

        jac_solver.JntToJac(jointpositions, jacobian);

        i++;

        ROS_INFO("Gravity vector (G):");
        for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
        {
          ROS_INFO("%f\t", G(i));
        }
        ROS_INFO("\n");

        test.linear.y = cartpos(3,2);

        joint_pub.publish(test);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// #include "sensor_msgs/JointState.h"
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/frames_io.hpp>
// #include <dasom_toolbox/dasom_joint.h>
// #include <dasom_toolbox/dasom_realsense_d435i.h>
// #include <dasom_toolbox/dasom_camera.h>
// #include <dasom_toolbox/dasom_tf2.h>
// #include <visualization_msgs/Marker.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Twist.h>

// #define PI 3.141592

// using namespace dasom;

// double joint = 0;
// double joint2 = 0;
// double i = 0;

// void Callback(const sensor_msgs::JointState &msg)
// {
//     joint = msg.effort[0];
//     joint2 = msg.effort[1] + 0.1*sin(3.14 * i * i - i);
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "try");
//     ros::NodeHandle nh;
//     ROS_WARN("try node start!");

//     // nh.setParam("/dasom_rviz_started", true);

//     // ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
//     ros::Publisher rs_c_pub_ = nh.advertise<sensor_msgs::Image>("/dasom/d435i/color", 10);
//     ros::Publisher rs_d_pub_ = nh.advertise<sensor_msgs::Image>("/dasom/d435i/depth", 10);
//     ros::Publisher joint_pub = nh.advertise<geometry_msgs::Twist>("/joint1", 100);
//     ros::Subscriber subs_ = nh.subscribe("/dasom/joint_states", 10, Callback);
//     ros::Subscriber sub;
    
//     // For DasomCam
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise("/dasom/camera_image", 1);

//     ros::Rate loop_rate(200);

//     // double joint1 = 0, joint2 = 0, joint3 = 0, joint4 = 0, joint5 = 0, joint6 = 0, 
//     //        base_joint_X = 0, base_joint_Y = 0, base_joint_Z = 0;
//     // double i = 0;
//     // double t = 0;

//     // sensor_msgs::JointState joint_states;

//     // Eigen::Vector3d point;

//     // point << 0, 0, 0;

//     // For DasomLPF
//     // DasomJoint ds_joint_(8, 8);
//     // DasomLPF ds_lpf_2(8);

//     // For DasomRealSense
//     // DasomRealSense ds_rs_(point, rs_c_pub_, rs_d_pub_);

//     // For DasomCam
//     // DasomCam ds_cam_(pub, 0);

//     // // For DasomTF2
//     // DasomTF2 ds_tf2_(sub,"/dasom/EE_cmd","world","joystickCMD");
//     double law_data = 0;
//     double raw_data = 0;
//     while (ros::ok())
//     {   
//         // For DasomLPF
//         // geometry_msgs::Twist msg;
//         // law_data = 0.1*sin(50 * i * i - i) + 10 * sin(0.005 * 3.14 * 2 / 4 * i);

//         // raw_data = 0.1*cos(50 * i * i - i) + 10 * cos(0.005 * 3.14 * 2 / 4 * i);
//         // // msg.header.stamp = ros::Time::now();
//         // msg.linear.x = ds_lpf_.updateLPF(0.005, law_data);
//         // msg.linear.y = law_data;

//         // msg.angular.x = ds_lpf_2.updateLPF(0.005, raw_data);
//         // msg.angular.y = raw_data;

//         // joint_pub.publish(msg);

//         // For DasomRealSense
//         // ds_rs_.test();
//         // ds_rs_.updateCamera();

//         // For DasomCam
//         // ds_cam_.UpdateCameraCommand(point);
//         // ds_cam_.DetectLightBulb();
        
//         //update joint_state
//         // joint_states.header.stamp = ros::Time::now();
//         // joint_states.name.resize(9);
//         // joint_states.position.resize(9);
//         // joint_states.name[0] = "id_1";
//         // joint_states.position[0] = joint1;
//         // joint_states.name[1] = "id_2";
//         // joint_states.position[1] = joint2;
//         // joint_states.name[2] = "id_3";
//         // joint_states.position[2] = joint3;
//         // joint_states.name[3] = "id_4";
//         // joint_states.position[3] = joint4;
//         // joint_states.name[4] = "id_5";
//         // joint_states.position[4] = joint5;
//         // joint_states.name[5] = "id_6";
//         // joint_states.position[5] = joint6;
//         // joint_states.name[6] = "base_joint_X";
//         // joint_states.position[6] = base_joint_X;
//         // joint_states.name[7] = "base_joint_Y";
//         // joint_states.position[7] = base_joint_Y;
//         // joint_states.name[8] = "base_joint_Z";
//         // joint_states.position[8] = base_joint_Z;

//         // t = i / 100;
//         // joint1 = t;
//         // joint2 = 1 + t;
//         // joint3 = -t;
//         // joint4 = t;
//         // joint5 = 0;
//         // joint6 = 0;
//         // base_joint_X = sin(4*t-PI);
//         // base_joint_Y = sin(4*t-PI);
//         // base_joint_Z = sin(2*t);

//         // joint_pub.publish(joint_states);

//         i++;

//         // ROS_ERROR("i: %lf", i);
//         // if(i > 157) break;

//         // ROS_INFO("joint1: %lf", joint1);
//         // ROS_INFO("joint2: %lf", joint2);
//         // ROS_INFO("joint3: %lf", joint3);
//         // ROS_INFO("joint4: %lf", joint4);
//         // ROS_WARN("==========================");

//         // ros::spinOnce();
//         // loop_rate.sleep();
        
//     }


//     ///////////////////////////////////////////////////////////
//     //          3D Marker 그리기, RViz에서 확인 가능          //
//     ///////////////////////////////////////////////////////////
//     // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("cube_marker", 1);

//     // visualization_msgs::Marker marker;
//     // marker.header.frame_id = "world";
//     // marker.header.stamp = ros::Time::now();
//     // marker.ns = "cube";
//     // marker.id = 0;
//     // marker.type = visualization_msgs::Marker::LINE_LIST;
//     // marker.action = visualization_msgs::Marker::ADD;
//     // marker.scale.x = 0.02;

//     // std::vector<geometry_msgs::Point> cube_vertices;
//     // for (int i = 0; i < 8; ++i) {
//     //     geometry_msgs::Point vertex;
//     //     vertex.x = i % 2;
//     //     vertex.y = (i / 2) % 2;
//     //     vertex.z = i / 4;
//     //     cube_vertices.push_back(vertex);
//     // }

//     // std::vector<std::pair<int, int>> cube_edges = {
//     //     {0, 1}, {1, 2}, {2, 3}, {3, 0},
//     //     {4, 5}, {5, 6}, {6, 7}, {7, 4},
//     //     {0, 4}, {1, 5}, {2, 6}, {3, 7}
//     // };

//     // for (const auto& edge : cube_edges) {
//     //     marker.points.push_back(cube_vertices[edge.first]);
//     //     marker.points.push_back(cube_vertices[edge.second]);
//     // }

//     // marker.color.r = 1.0;
//     // marker.color.g = 1.0;
//     // marker.color.b = 1.0;
//     // marker.color.a = 1.0;

//     // ros::Rate loop_rate(10);
//     // while (ros::ok()) {
//     //     marker_pub.publish(marker);
//     //     ros::spinOnce();
//     //     loop_rate.sleep();
//     // }

//     ///////////////////////////////////////////
//     //       3D 큐브 그리고 좌표 받아오기      //
//     ///////////////////////////////////////////
//     // cv::Mat frame(600, 800, CV_8UC3, cv::Scalar(0, 0, 0)); // 이미지 생성

//     // // 큐브의 3D 좌표
//     // std::vector<cv::Point3f> cubeVertices = {
//     //     {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
//     //     {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
//     // };

//     // // 연결선 인덱스
//     // std::vector<std::pair<int, int>> cubeEdges = {
//     //     {0, 1}, {1, 2}, {2, 3}, {3, 0},
//     //     {4, 5}, {5, 6}, {6, 7}, {7, 4},
//     //     {0, 4}, {1, 5}, {2, 6}, {3, 7}
//     // };

//     // // X, Y, Z 축 회전 행렬 생성
//     // cv::Mat rotationMatrixX = (cv::Mat_<float>(3, 3) <<
//     //     1, 0, 0,
//     //     0, cos(CV_PI / 4), -sin(CV_PI / 4),
//     //     0, sin(CV_PI / 4), cos(CV_PI / 4)
//     // );
//     // cv::Mat rotationMatrixY = (cv::Mat_<float>(3, 3) <<
//     //     cos(CV_PI / 4), 0, sin(CV_PI / 4),
//     //     0, 1, 0,
//     //     -sin(CV_PI / 4), 0, cos(CV_PI / 4)
//     // );
//     // cv::Mat rotationMatrixZ = (cv::Mat_<float>(3, 3) <<
//     //     cos(CV_PI / 4), -sin(CV_PI / 4), 0,
//     //     sin(CV_PI / 4), cos(CV_PI / 4), 0,
//     //     0, 0, 1
//     // );

//     // // 큐브의 회전을 적용 (X, Y, Z 축 순차적으로 회전) & 새로운 3D 좌표 얻기
//     // std::vector<cv::Point3f> rotatedCubeVertices;
//     // for (auto& vertex : cubeVertices) {
//     //     cv::Mat rotatedPoint = rotationMatrixZ * (rotationMatrixY * (rotationMatrixX * (cv::Mat_<float>(3, 1) << vertex.x, vertex.y, vertex.z)));
//     //     vertex.x = rotatedPoint.at<float>(0);
//     //     vertex.y = rotatedPoint.at<float>(1);
//     //     vertex.z = rotatedPoint.at<float>(2);
//     //     rotatedCubeVertices.push_back(cv::Point3f(rotatedPoint.at<float>(0), rotatedPoint.at<float>(1), rotatedPoint.at<float>(2)));
//     // }

//     // // 3D 큐브를 그리기 위해 연결선 그리기
//     // for (const auto& edge : cubeEdges) {
//     //     cv::line(frame, cv::Point(cubeVertices[edge.first].x * 100 + 400, cubeVertices[edge.first].y * 100 + 300),
//     //                      cv::Point(cubeVertices[edge.second].x * 100 + 400, cubeVertices[edge.second].y * 100 + 300),
//     //                      cv::Scalar(0, 0, 255), 2);
//     // }

//     // // 회전된 3D 좌표 출력
//     // for (const auto& vertex : rotatedCubeVertices) {
//     //     std::cout << "X: " << vertex.x << ", Y: " << vertex.y << ", Z: " << vertex.z << std::endl;
//     // }

//     // cv::imshow("3D Cube Visualization", frame);
//     // cv::waitKey(0);


//     return 0;
// }