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
#include <opencv2/opencv.hpp>

cv::Mat image;

void mouseCallback(int event, int x, int y, int flags, void* param) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        // 클릭한 지점의 픽셀 값을 읽어옴
        cv::Vec3b hsv = image.at<cv::Vec3b>(y, x);

        // HSV 값을 출력
        std::cout << "Clicked HSV values at (" << x << ", " << y << "): "
                  << "H=" << static_cast<int>(hsv[0]) << ", "
                  << "S=" << static_cast<int>(hsv[1]) << ", "
                  << "V=" << static_cast<int>(hsv[2]) << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "try");
    ros::NodeHandle nh;

    ros::Rate rate(200);

    // 이미지 파일 읽기
    cv::Mat image = cv::imread("/home/choisol/image.png");

    if (image.empty()) {
        std::cerr << "Image not found or cannot be read." << std::endl;
        return -1;
    }

    // // 이미지 창 생성 및 마우스 콜백 함수 등록
    // cv::namedWindow("Image", cv::WINDOW_NORMAL);  // 크기 조절을 위해 WINDOW_NORMAL 플래그 사용
    // cv::resizeWindow("Image", image.cols / 2, image.rows / 2);  // 이미지 크기를 50%로 조절
    // cv::setMouseCallback("Image", mouseCallback);

    // // 무한 루프
    // while (true) {
    //     // 이미지를 창에 표시
    //     cv::imshow("Image", image);

    //     // 키 입력을 기다림
    //     int key = cv::waitKey(10);

    //     // 'q' 키를 누르면 종료
    //     if (key == 'q') {
    //         break;
    //     }
    // }

    // 이미지를 HSV 색 공간으로 변환
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    // 색상 범위 지정 (노란색 예시)
    cv::Scalar lower_yellow = cv::Scalar(30, 80, 100);  // HSV에서의 하한 범위
    cv::Scalar upper_yellow = cv::Scalar(50, 255, 255);  // HSV에서의 상한 범위

    // 지정된 색상 범위에 속하는 픽셀을 흰색으로 마스킹
    cv::Mat mask;
    cv::inRange(hsvImage, lower_yellow, upper_yellow, mask);

    // 원본 이미지에서 마스크를 적용하여 노란색을 추출
    cv::Mat result;
    cv::bitwise_and(image, image, result, mask);

    // 이미지 크기 조절 (가로 800, 세로 자동 조절)
    cv::resize(image, image, cv::Size(700, 850), 0, 0, cv::INTER_LINEAR);
    cv::resize(mask, mask, cv::Size(700, 850), 0, 0, cv::INTER_LINEAR);
    cv::resize(result, result, cv::Size(700, 850), 0, 0, cv::INTER_LINEAR);

    // 원본 이미지, 마스크, 추출된 이미지를 각각 표시
    cv::imshow("Original Image", image);
    cv::imshow("Mask", mask);
    cv::imshow("Yellow Color Extraction", result);

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}





// #include <geometry_msgs/PoseStamped.h>

// int main(int argc, char **argv) 
// {
//     ros::init(argc, argv, "try");
//     ros::NodeHandle nh;

//     ros::Rate rate(200);

//     ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/dasomPalletrone/world", 1);

//     geometry_msgs::PoseStamped msg;

//     double i = 0;

//     ros::Time recording_start_time = ros::Time::now();

//     while(ros::ok())
//     {
//         ros::Duration elapsed_time = ros::Time::now() - recording_start_time;
//         msg.header.stamp = ros::Time(elapsed_time.toSec());
//         // msg.header.stamp.sec = ros::Time::now().toSec;

//         msg.pose.position.x = 5 * sin(i/200);
//         msg.pose.position.y = 5 * cos(i/200);
//         msg.pose.position.z = 3;
//         msg.pose.orientation.x = 0;
//         msg.pose.orientation.y = 0;
//         msg.pose.orientation.z = 0;
//         msg.pose.orientation.w = 1;

//         i++;

//         pub.publish(msg);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }


// #include "ros/ros.h"
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>
// #include <Eigen/QR>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>
// #include <kdl/frames.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainidsolver_recursive_newton_euler.hpp>
// #include <stdio.h>
// #include <iostream>
// #include <sensor_msgs/JointState.h>
// #include <geometry_msgs/Wrench.h>
// #include <dasom_toolbox/dasom_workbench.h>
// #include <dasom_controllers/admittanceSRV.h>
// #include <dasom_controllers/bandpassSRV.h>
 
// using namespace KDL;

// template<typename Derived>
// Derived dampedPinv(const Eigen::MatrixBase<Derived>& a, double rho = 1e-4) 
// {
// 	return a.transpose() * (a * a.transpose() + rho * rho * Eigen::MatrixBase<Derived>::Identity(a.rows(), a.rows())).inverse();
// }

// KDL::Chain chain;

// double num_jnt = 3;

// KDL::JntArray q(num_jnt);
// KDL::JntArray qdot(num_jnt);
// KDL::JntArray qdotdot(num_jnt);
// KDL::JntArray tau(num_jnt);

// // Eigen::VectorXd jnt_p;
// // Eigen::VectorXd jnt_v;
// // Eigen::VectorXd jnt_e;

// // double num_jnt = 6;
// // jnt_p.resize(num_jnt);
// // jnt_v.resize(num_jnt);
// // jnt_e.resize(num_jnt);

// // void cmdgenerator()
// // {
// //     sensor_msgs::JointState jnt;
// //     double i = 0;
// //     double move;

// //     move = sin(i/100) + 1.57;
// //     ROS_ERROR("%lf",move);

// //     jnt.position.push_back(0);
// //     jnt.position.push_back(0);

// //     joint_cmd_pub.publish(jnt);
// // }

// double wl = 1;
// double wh = 2;
// double w = sqrt(wl * wh);
// double Q = w / (wh - wl);
// Eigen::Matrix2d bp_A;
// Eigen::Vector2d bp_B;
// Eigen::Vector2d bp_C;
// double bp_D;
// Eigen::Vector2d bp_X1;
// Eigen::Vector2d bp_X2;
// Eigen::Vector2d bp_X3;
// Eigen::Vector2d bp_X_dot1;
// Eigen::Vector2d bp_X_dot2;
// Eigen::Vector2d bp_X_dot3;
// Eigen::Vector3d bf_F_ext;

// // For admittance control
// double virtual_mass_x;
// double virtual_damper_x;
// double virtual_spring_x;
// double virtual_mass_y;
// double virtual_damper_y;
// double virtual_spring_y;
// double virtual_mass_z;
// double virtual_damper_z;
// double virtual_spring_z;

// bool bandpassCallback(dasom_controllers::bandpassSRV::Request & req,
//                       dasom_controllers::bandpassSRV::Response & res)
// {
//   wl = req.wl; // ㅁㅁㅁㅁ 이거 잘 숫자 넣어서 튜닝해보십쇼
//   wh = req.wh; // 컷오프 프리퀀시 lower랑 higher임.
//   double w = sqrt(wl * wh);
//   double Q = w / (wh - wl);

//   ROS_INFO("wl = %lf", wl);
//   ROS_INFO("wh = %lf", wh);
  
//   //pf//
//   bp_A << - w/ Q, - w*w,
//               1,      0;

//   bp_B << 1, 0;
//   bp_B.transpose();
//   bp_C << w/ Q, 0;
//   bp_D = 0;
//   bp_X1 << 0, 0;
//   bp_X1.transpose();
//   bp_X2 << 0, 0;
//   bp_X2.transpose();
//   bp_X3 << 0, 0;
//   bp_X3.transpose();

//   return true;
// }

// // bool admittanceCallback(dasom_controllers::admittanceSRV::Request  &req,
// //                         dasom_controllers::admittanceSRV::Response &res)
// // {
// //   virtual_mass_x = req.x_m;
// //   virtual_damper_x = req.x_d;
// //   virtual_spring_x = req.x_k;
  
// //   virtual_mass_y = req.y_m;
// //   virtual_damper_y = req.y_d;
// //   virtual_spring_y = req.y_k;

// //   virtual_mass_z = req.z_m;
// //   virtual_damper_z = req.z_d;
// //   virtual_spring_z = req.z_k;

// //   initializeAdmittance();

// //   X_from_model_matrix << 0, 0;
// //   X_dot_from_model_matrix << 0, 0;
// //   Y_from_model_matrix << 0, 0;
// //   Y_dot_from_model_matrix << 0, 0;
// //   Z_from_model_matrix << 0, 0;
// //   Z_dot_from_model_matrix << 0, 0;

// //   return true;
// // }


// void jointCallback(const sensor_msgs::JointState &msg)
// {
//     for (unsigned int i = 0; i < num_jnt; ++i) 
//     {
//         q(i) = msg.position.at(i);
//         qdot(i) = msg.velocity.at(i);
//         tau(i) = msg.effort.at(i);
//     }
// }

// int main( int argc, char** argv )
// {
//     ros::init(argc, argv, "try");
//     ros::NodeHandle nh;
//     ROS_WARN("try node start!");

//     ros::Rate loop_rate(20);

//     ros::Publisher force_pub_ = nh.advertise<geometry_msgs::Wrench>("ext_force", 10);
//     ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/dasom/goal_dynamixel_position", 10);
//     ros::Subscriber joint_sub_ = nh.subscribe("/joint_states", 10, &jointCallback);
//     ros::ServiceServer bandpass_srv_ = nh.advertiseService("/bandpass_srv", &bandpassCallback);

//     dasom::DasomWorkbench ds_wb_;

//     sensor_msgs::JointState jnt;
//     double i = 0;
//     double move;

//     double time_i = ros::Time::now().toSec();
//     double time_f;
//     double time_loop;

//     geometry_msgs::Wrench ext;

//     //Definition of a kinematic chain & add segments to the chain
//     // KDL::Chain chain;

//     double l1 = 0.05465;
//     double l2 = 0.1549;
//     double l3 = 0.116;
//     double l4 = 0.06183;
//     double l5 = 0.05075;
//     double l6 = 0.0613;
//     double l7 = 0.115;

//     // joint segment
//     std::vector<KDL::Vector> link_lengths;
//     std::vector<KDL::Vector> link_cogs;
//     std::vector<KDL::RotationalInertia> link_inertias;

//     link_lengths = {
//                         KDL::Vector(0.0,0.0,l1), // link1
//                         KDL::Vector(0.0,l2,0.0), // link2
//                         KDL::Vector(0.0,l3,0.0), // link3
//                         KDL::Vector(0.0,l5,-l4), // link4
//                         KDL::Vector(0.0,0.0,l6), // link5
//                         KDL::Vector(0.0,l7,0.0)  // link6
//                    };

//     link_cogs = {
//                     KDL::Vector(0,0,0.027325), // link1
//                     KDL::Vector(0.00039,-0.03411,0.0), // link2
//                     KDL::Vector(-0.00001,-0.06095,0.00226), // link3
//                     KDL::Vector(-0.00036,-0.03577,0.02293), // link4
//                     KDL::Vector(0.0434,0.00368,-0.02326), // link5
//                     KDL::Vector(-0.04427,-0.06193,-0.00627)  // link6
//                 };

//     // link_inertias = KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz);
//     link_inertias = {
//                         1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link1
//                         1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link2
//                         1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link3
//                         1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link4
//                         1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0), // link5
//                         1.0e-09 * KDL::RotationalInertia(0, 0, 0, 0, 0, 0)  // link6
//                     };

//     // Set Joint Configuration
//     // 0.0
//     chain.addSegment(Segment(Joint(Joint::RotZ), Frame(link_lengths[0]),
//                              RigidBodyInertia(0.5, link_cogs[0], link_inertias[0])));
//     // 0.0 0.263367
//     chain.addSegment(Segment(Joint(Joint::RotX), Frame(link_lengths[1]),
//                              RigidBodyInertia(0.22226, link_cogs[1], link_inertias[1])));
//     // 0.0 0.577486	0.117183	
//     chain.addSegment(Segment(Joint(Joint::RotX), Frame(link_lengths[2]),
//                              RigidBodyInertia(0.0476, link_cogs[2], link_inertias[2])));
//     // // 0.0 0.641148	0.146350 0.000080
//     // chain.addSegment(Segment(Joint(Joint::RotY), Frame(link_lengths[3]),
//     //                          RigidBodyInertia(0.0227, link_cogs[3], link_inertias[3])));
//     // // 0.0 0.991254	0.329760 -0.046625 0.000000	
//     // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(link_lengths[4]),
//     //                          RigidBodyInertia(0.1097, link_cogs[4], link_inertias[4])));
//     // // 0.0 2.110560	0.986372 0.085612 0.000000 0.158523	
//     // chain.addSegment(Segment(Joint(Joint::RotX), Frame(link_lengths[5]),
//     //                          RigidBodyInertia(0.30449, link_cogs[5], link_inertias[5])));
 
//     // Create solver based on kinematic chain
//     ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
//     // Create joint array
//     KDL::JntArray tauHCGa(chain.getNrOfJoints());
//     KDL::JntArray tauHCG(chain.getNrOfJoints());
//     KDL::JntArray C(chain.getNrOfJoints()); //coriolis matrix
//     KDL::JntArray G(chain.getNrOfJoints()); //gravity matrix
//     KDL::Wrenches f(chain.getNrOfSegments());
//     KDL::Vector grav(0.0,0.0,-9.81);
//     KDL::JntSpaceInertiaMatrix H(chain.getNrOfJoints()); //inertiamatrix H=square matrix of size= number of joints
//     KDL::ChainDynParam chaindynparams(chain,grav);   
 
//     // Assign some values to the joint positions
//     // for(unsigned int i=0;i<chain.getNrOfJoints();i++){
//     //     float myinput;
//     //     printf ("Enter the position of joint %i: ",i);
//     //     scanf ("%e",&myinput);
//     //     q(i)=(double)myinput;
//     // }

//     double t;

//     bp_A << - w/ Q, - w*w,
//               1,      0;

//     bp_B << 1, 0;
//     bp_B.transpose();
//     bp_C << w/ Q, 0;
//     bp_D = 0;
//     bp_X1 << 0, 0;
//     bp_X1.transpose();
//     bp_X2 << 0, 0;
//     bp_X2.transpose();
//     bp_X3 << 0, 0;
//     bp_X3.transpose();

//     // printf("AAAAAAA");
//     while(ros::ok())
//     {
//         move = sin(t/100) + 1.57;
//         ROS_ERROR("%lf", move);

//         time_f = ros::Time::now().toSec();
//         time_loop = time_f - time_i;
//         time_i = ros::Time::now().toSec();
        
//         // 관절 각도, 속도, 가속도 설정 (예시로 0으로 초기화)
//         for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
//             // printf("BBBBBBB");
//             // q(i) = jnt_p[i];
//             // qdot(i) = jnt_v[i];
//             qdotdot(i) = 0.0;
//             // tau(i) = jnt_e[i];
//         }
//         printf("Joint2 position = %lf\n", q(1));
    
//         // Create the frame that will contain the results
//         KDL::Frame cartpos;    
    
//         // Calculate forward position kinematics
//         bool kinematics_status;
//         kinematics_status = fksolver.JntToCart(q,cartpos);
//         if(kinematics_status>=0){
//             std::cout << cartpos <<std::endl;
//             // printf("%s \n","Succes, thanks KDL!");
//         }
//         else{
//             // printf("%s \n","Error: could not calculate forward kinematics :(");
//         }

//         // calculation of the HCG matrices
//         chaindynparams.JntToMass(q,H);
//         chaindynparams.JntToCoriolis(q,qdot,C);
//         chaindynparams.JntToGravity(q,G);

//         // ROS_INFO("Mass matrix (H):");
//         // for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
//         // {
//         //   for (unsigned int j = 0; j < chain.getNrOfJoints(); ++j)
//         //   {
//         //     ROS_INFO("%f\t", H(i, j));
//         //   }
//         //   ROS_INFO("\n");
//         // }

//         // ROS_INFO("Coriolis matrix (C):");
//         // for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
//         // {
//         //   ROS_INFO("%f\t", C(i));
//         // }
//         // ROS_INFO("\n");

//         // ROS_INFO("Gravity vector (G):");
//         // for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
//         // {
//         //   ROS_INFO("%f\t", G(i));
//         // }
//         // ROS_INFO("\n");

//         // calculation of the Jacobian matrix
//         KDL::ChainJntToJacSolver jac_solver(chain);
//         KDL::Jacobian jacobian(chain.getNrOfJoints());
//         jac_solver.JntToJac(q, jacobian);

//         // std::cout << "Jacobian 행렬:" << std::endl;
//         for (unsigned int i = 0; i < jacobian.columns(); ++i) {
//             for (unsigned int j = 0; j < jacobian.rows(); ++j) {
//                 // std::cout << jacobian(j, i) << " ";
//             }
//             // std::cout << std::endl;
//         }

//         // Eigen 행렬로 변환
//         Eigen::MatrixXd eigen_jacobian(6, chain.getNrOfJoints());
//         for (int i = 0; i < 6; ++i) {
//             for (int j = 0; j < chain.getNrOfJoints(); ++j) {
//                 eigen_jacobian(i, j) = jacobian(i, j);
//             }
//         }
        
//         // Jacobian 행렬의 Transpose 계산
//         Eigen::MatrixXd transpose_jacobian = eigen_jacobian.transpose();

//         // Jacobian 행렬의 Inverse 계산
//         Eigen::MatrixXd transpose_inverse_jacobian = dampedPinv(transpose_jacobian);
//         // Eigen::MatrixXd transpose_inverse_jacobian = transpose_jacobian.inverse();

//         // Eigen 행렬을 KDL::Jacobian으로 변환
//         KDL::Jacobian kdl_jacobian(chain.getNrOfJoints()); // Jacobian 행렬 초기화
//         for (int i = 0; i < 6; ++i) {
//             for (int j = 0; j < chain.getNrOfJoints(); ++j) {
//                 kdl_jacobian(i, j) = transpose_inverse_jacobian(i, j);
//             }
//         }

//         Eigen::VectorXd jnt_p(6);
//         for (int i = 0; i < chain.getNrOfJoints(); ++i)
//         {
//             jnt_p[i] = q(i);
//         }

//         Eigen::MatrixXd Jac(6, chain.getNrOfJoints());
//         Jac = ds_wb_.Jacobian(jnt_p);

//         // calculation of the force estimation
//         KDL::ChainIdSolver_RNE id_solver(chain, grav);

//         // 가상의 힘 및 토크 생성
//         // KDL::Wrenches virtual_force(chain.getNrOfJoints()); // 가상의 힘 및 토크를 저장하기 위한 객체
//         // virtual_force[5].force.x(10.0); // x 축 방향으로 10 N의 힘을 가상으로 적용
//         // virtual_force[5].torque.z(1.0); // z 축 주변으로 1 Nm의 토크를 가상으로 적용


//         // 토크 및 힘 계산
//         id_solver.CartToJnt(q,qdot,qdotdot,f,G);
//         // id_solver.CartToJnt(q,qdot,qdotdot,virtual_force,tau);

//         // 추정된 힘 및 토크 출력
//         // std::cout << "End-Effector Force (Fx, Fy, Fz): " << f.force.x() << ", " << f.force.y() << ", " << f.force.z() << std::endl;
//         // std::cout << "End-Effector Torque (Tx, Ty, Tz): " << f.torque.x() << ", " << f.torque.y() << ", " << f.torque.z() << std::endl;

//         ROS_WARN("==============================");
//         for (size_t i = 0; i < f.size(); ++i) {
//         // std::cout << "Force " << i << " (Fx, Fy, Fz): " << f[i].force.x() << ", " << f[i].force.y() << ", " << f[i].force.z() << std::endl;
//         // std::cout << "Force " << i << " (Fx, Fy, Fz): " << virtual_force[i].force.x() << ", " << virtual_force[i].force.y() << ", " << virtual_force[i].force.z() << "///";
//         // std::cout << "Torque " << i << " (Tx, Ty, Tz): " << f[i].torque.x() << ", " << f[i].torque.y() << ", " << f[i].torque.z() << std::endl;
//         // std::cout << "Torque " << i << " (Tx, Ty, Tz): " << virtual_force[i].torque.x() << ", " << virtual_force[i].torque.y() << ", " << virtual_force[i].torque.z() << std::endl;
//         }

//         for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
//             // std::cout << "Joint " << i << " Torque: " << tau(i) << std::endl;
//         }

//         KDL::JntArray F_ext(chain.getNrOfJoints());
//         for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
//             for (unsigned int j = 0; j < chain.getNrOfJoints(); ++j) {
//             F_ext(i) = kdl_jacobian(i,j) * (tau(i) - C(i) - G(i));
//             }
//         }

//         Eigen::Vector3d F_ext_Eigen(3);
//         for (unsigned int i = 0; i < 3; ++i) {
//             F_ext_Eigen[i] = F_ext(i);
//         }    
            
//         // std::cout << "Force " << F_ext(0) << "  " << F_ext(1) << "  " << F_ext(2) << "  " << F_ext(3) << "  " << F_ext(4) << "  " << F_ext(5) << std::endl;

//         Eigen::MatrixXd jac(6, chain.getNrOfJoints());
//         for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
//             for (unsigned int j = 0; j < chain.getNrOfJoints(); ++j) {
//                 jac(i,j) = Jac(i,j) - eigen_jacobian(i,j);
//             }
//         }

//         bp_X_dot1 = bp_A * bp_X1 + bp_B * F_ext_Eigen[0];
//         bp_X1 += bp_X_dot1 * time_loop;
//         bf_F_ext[0] = bp_C.dot(bp_X1) + F_ext_Eigen[0] * bp_D;

//         bp_X_dot2 = bp_A * bp_X2 + bp_B * F_ext_Eigen[1];
//         bp_X2 += bp_X_dot2 * time_loop;
//         bf_F_ext[1] = bp_C.dot(bp_X2) + F_ext_Eigen[1] * bp_D;

//         bp_X_dot3 = bp_A * bp_X3 + bp_B * F_ext_Eigen[2];
//         bp_X3 += bp_X_dot3 * time_loop;
//         bf_F_ext[2] = bp_C.dot(bp_X3) + F_ext_Eigen[2] * bp_D;
        
//         std::cout<<F_ext_Eigen<<std::endl;
//         // ROS_INFO("==============================");
//         // std::cout<<eigen_jacobian<<std::endl;
//         // ROS_INFO("==============================");
//         // std::cout<<jac<<std::endl;

//         // ext.force.x = tau(0);
//         // ext.force.y = tau(1);
//         // ext.force.z = tau(2);
//         // ext.torque.x = C(0) + G(0);
//         // ext.torque.y = C(1) + G(1);
//         // ext.torque.z = C(2) + G(2);

//         ext.force.x = bf_F_ext(0);
//         ext.force.y = bf_F_ext(1);
//         ext.force.z = bf_F_ext(2);
//         ext.torque.x = F_ext(0);
//         ext.torque.y = F_ext(1);
//         ext.torque.z = F_ext(2);

//         force_pub_.publish(ext);

//         t++;

//         jnt.header.stamp = ros::Time::now();
//         jnt.name.resize(3);
//         jnt.position.resize(3);
//         jnt.name[0] = "id_1";
//         jnt.position[0] = 0;
//         jnt.name[1] = "id_2";
//         jnt.position[1] = 1; //move;
//         jnt.name[2] = "id_3";
//         jnt.position[2] = -0.6;

//         joint_cmd_pub.publish(jnt);

//         loop_rate.sleep();
//         ros::spinOnce();
//     }
// }


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

// #include "ros/ros.h"
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
// #include <geometry_msgs/WrenchStamped.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include "tf/transform_datatypes.h"

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
//     ros::Publisher joint_pub = nh.advertise<geometry_msgs::Twist>("/palletrone/battery", 100);
//     ros::Publisher pubpub = nh.advertise<geometry_msgs::WrenchStamped>("/dasom/estimated_force", 100);
//     ros::Subscriber subs_ = nh.subscribe("/dasom/joint_states", 10, Callback);
//     ros::Subscriber sub;
    
//     // For DasomCam
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise("/dasom/camera_image", 1);

//     ros::Rate loop_rate(200);

//     double joint1 = 0, joint2 = 0, joint3 = 0, joint4 = 0, joint5 = 0, joint6 = 0, 
//            base_joint_X = 0, base_joint_Y = 0, base_joint_Z = 0;
//     double i = 0;
//     double t = 0;

//     // sensor_msgs::JointState joint_states;

//     Eigen::Vector3d point;

//     point << 0, 0, 0;

//     double roll, pitch, yaw;

//     geometry_msgs::PoseStamped msg;
//     geometry_msgs::WrenchStamped joint_states;

//     msg.pose.orientation.x = 0;
//     msg.pose.orientation.y = 0;
//     msg.pose.orientation.z = 0.7;
//     msg.pose.orientation.w = 0.7;

//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(msg.pose.orientation, quat);

//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

//     ROS_INFO("roll, pitch, yaw = %lf, %lf, %lf", roll, pitch, yaw);

//     roll = PI/2;
//     pitch = 0;
//     yaw = 0;

//     // (PI/2, 0, 0) -> (0.707, 0, 0, 0.707)

//     quat.setRPY(roll, pitch, yaw);

//     msg.pose.orientation.x = quat.x();
//     msg.pose.orientation.y = quat.y();
//     msg.pose.orientation.z = quat.z();
//     msg.pose.orientation.w = quat.w();

//     ROS_INFO("x, y, z, w = %lf, %lf, %lf, %lf", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);

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
//         geometry_msgs::Twist msg;
//         // law_data = 0.1*sin(50 * i * i - i) + 10 * sin(0.005 * 3.14 * 2 / 4 * i);

//         // raw_data = 0.1*cos(50 * i * i - i) + 10 * cos(0.005 * 3.14 * 2 / 4 * i);
//         // msg.header.stamp = ros::Time::now();
//         msg.linear.x = 15 + 2 * sin(i/200);
//         // msg.linear.y = law_data;

//         // msg.angular.x = ds_lpf_2.updateLPF(0.005, raw_data);
//         // msg.angular.y = raw_data;

//         joint_pub.publish(msg);

//         // For DasomRealSense
//         // ds_rs_.test();
//         // ds_rs_.updateCamera();

//         // For DasomCam
//         // ds_cam_.UpdateCameraCommand(point);
//         // ds_cam_.DetectLightBulb();
        
//         //update joint_state
//         joint_states.header.stamp = ros::Time::now();
//         // joint_states.name.resize(9);
//         // joint_states.position.resize(9);
//         // joint_states.name[0] = "id_1";
//         joint_states.wrench.force.x = joint1;
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

//         t = i / 100;
//         joint1 = 3*sin(t);
//         // joint2 = 1 + t;
//         // joint3 = -t;
//         // joint4 = t;
//         // joint5 = 0;
//         // joint6 = 0;
//         // base_joint_X = sin(4*t-PI);
//         // base_joint_Y = sin(4*t-PI);
//         // base_joint_Z = sin(2*t);

//         pubpub.publish(joint_states);

//         i++;

//         // ROS_ERROR("i: %lf", i);
//         // if(i > 157) break;

//         // ROS_INFO("joint1: %lf", joint1);
//         // ROS_INFO("joint2: %lf", joint2);
//         // ROS_INFO("joint3: %lf", joint3);
//         // ROS_INFO("joint4: %lf", joint4);
//         // ROS_WARN("==========================");

//         ros::spinOnce();
//         loop_rate.sleep();
        
//     }
// }

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