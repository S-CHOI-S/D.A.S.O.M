#include "../include/tf_broadcaster.h"

TFBroadcaster::TFBroadcaster()
: node_handle_("")
{
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();

  ds_jnt_ = new DasomJoint(8, 8);

  optitrackQuat.resize(7);
  optitrackQuat_lpf.resize(7);
}

TFBroadcaster::~TFBroadcaster()
{
  ROS_INFO("Bye TFBroadcaster!");
  ros::shutdown();
}

void TFBroadcaster::testCallback(const geometry_msgs::Twist& msg)
{
    // static tf2_ros::StaticTransformBroadcaster br_pose;
    // geometry_msgs::TransformStamped transformStamped_pose;

    // transformStamped_pose.header.stamp = ros::Time::now();
    // transformStamped_pose.header.frame_id = "world";
    // transformStamped_pose.child_frame_id = "haptic";
    // transformStamped_pose.transform.translation.x = msg.linear.x;
    // transformStamped_pose.transform.translation.y = msg.linear.y;
    // transformStamped_pose.transform.translation.z = msg.linear.z;

    // tf::Quaternion quat;
    // quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

    // transformStamped_pose.transform.rotation.x = quat.x();
    // transformStamped_pose.transform.rotation.y = quat.y();
    // transformStamped_pose.transform.rotation.z = quat.z();
    // transformStamped_pose.transform.rotation.w = quat.w();
    
    // br_pose.sendTransform(transformStamped_pose);
}

void TFBroadcaster::joystickCallback(const geometry_msgs::Twist& msg)
{
    // static tf2_ros::StaticTransformBroadcaster br_pose;
    // geometry_msgs::TransformStamped transformStamped_pose;

    // transformStamped_pose.header.stamp = ros::Time::now();
    // transformStamped_pose.header.frame_id = "world";
    // transformStamped_pose.child_frame_id = "joystick_command";
    // transformStamped_pose.transform.translation.x = msg.linear.x;
    // transformStamped_pose.transform.translation.y = msg.linear.y;
    // transformStamped_pose.transform.translation.z = msg.linear.z;

    // tf::Quaternion quat;
    // quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

    // transformStamped_pose.transform.rotation.x = quat.x();
    // transformStamped_pose.transform.rotation.y = quat.y();
    // transformStamped_pose.transform.rotation.z = quat.z();
    // transformStamped_pose.transform.rotation.w = quat.w();
    
    // br_pose.sendTransform(transformStamped_pose);
}

void TFBroadcaster::world2palletrone(Eigen::VectorXd optitrackquat)
{
    // static tf2_ros::StaticTransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // optitrackQuat_lpf[0] = ds_jnt_->updateLPF(time_loop, optitrackquat[0]);
    // optitrackQuat_lpf[1] = ds_jnt_->updateLPF(time_loop, optitrackquat[1]);
    // optitrackQuat_lpf[2] = ds_jnt_->updateLPF(time_loop, optitrackquat[2]);
    // optitrackQuat_lpf[3] = ds_jnt_->updateLPF(time_loop, optitrackquat[3]);
    // optitrackQuat_lpf[4] = ds_jnt_->updateLPF(time_loop, optitrackquat[4]);
    // optitrackQuat_lpf[5] = ds_jnt_->updateLPF(time_loop, optitrackquat[5]);
    // optitrackQuat_lpf[6] = ds_jnt_->updateLPF(time_loop, optitrackquat[6]);

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "world";
    // transformStamped.child_frame_id = "paletrone";
    // transformStamped.transform.translation.x = optitrackQuat_lpf[0];
    // transformStamped.transform.translation.y = optitrackQuat_lpf[1];
    // transformStamped.transform.translation.z = optitrackQuat_lpf[2];

    // transformStamped.transform.rotation.x = optitrackQuat_lpf[3];
    // transformStamped.transform.rotation.y = optitrackQuat_lpf[4];
    // transformStamped.transform.rotation.z = optitrackQuat_lpf[5];
    // transformStamped.transform.rotation.w = optitrackQuat_lpf[6];

    // br.sendTransform(transformStamped);
}

void TFBroadcaster::optitrackCallback(const geometry_msgs::PoseStamped& msg)
// 기준: Z = 0.38
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "ppaalleettrroonnee";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    // double roll, pitch, yaw;
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // ROS_WARN("%lf, %lf, %lf", roll, pitch, yaw);
    
    br.sendTransform(transformStamped);
}


void TFBroadcaster::PoseCallback(const geometry_msgs::Twist& msg)
{
    // static tf2_ros::StaticTransformBroadcaster br_pose;
    // geometry_msgs::TransformStamped transformStamped_pose;

    // transformStamped_pose.header.stamp = ros::Time::now();
    // transformStamped_pose.header.frame_id = "paletrone";
    // transformStamped_pose.child_frame_id = "global_EE_pose";
    // transformStamped_pose.transform.translation.x = msg.linear.x;
    // transformStamped_pose.transform.translation.y = msg.linear.y;
    // transformStamped_pose.transform.translation.z = msg.linear.z;
    
    // tf::Quaternion quat;

    // quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

    // transformStamped_pose.transform.rotation.x = quat.x();
    // transformStamped_pose.transform.rotation.y = quat.y();
    // transformStamped_pose.transform.rotation.z = quat.z();
    // transformStamped_pose.transform.rotation.w = quat.w();
    
    // br_pose.sendTransform(transformStamped_pose);
}

void TFBroadcaster::globalgimbalCallback(const geometry_msgs::PoseStamped &msg)
{  
    // static tf2_ros::StaticTransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "world";
    // transformStamped.child_frame_id = "global_gimbal_tf";
    // transformStamped.transform.translation.x = msg.pose.position.x;
    // transformStamped.transform.translation.y = msg.pose.position.y;
    // transformStamped.transform.translation.z = msg.pose.position.z;

    // transformStamped.transform.rotation.x = msg.pose.orientation.x;
    // transformStamped.transform.rotation.y = msg.pose.orientation.y;
    // transformStamped.transform.rotation.z = msg.pose.orientation.z;
    // transformStamped.transform.rotation.w = msg.pose.orientation.w;

    // br.sendTransform(transformStamped);
}

void TFBroadcaster::gimbalCallback(const geometry_msgs::PoseStamped& msg)
{
    // static tf2_ros::StaticTransformBroadcaster br_gimbal;
    // geometry_msgs::TransformStamped transformStamped_pose;

    // transformStamped_pose.header.stamp = ros::Time::now();
    // transformStamped_pose.header.frame_id = "paletrone";
    // transformStamped_pose.child_frame_id = "gimbal_tf";
    // transformStamped_pose.transform.translation.x = msg.pose.position.x;
    // transformStamped_pose.transform.translation.y = msg.pose.position.y;
    // transformStamped_pose.transform.translation.z = msg.pose.position.z;

    // transformStamped_pose.transform.rotation.x = msg.pose.orientation.x;
    // transformStamped_pose.transform.rotation.y = msg.pose.orientation.y;
    // transformStamped_pose.transform.rotation.z = msg.pose.orientation.z;
    // transformStamped_pose.transform.rotation.w = msg.pose.orientation.w;
    // br_gimbal.sendTransform(transformStamped_pose);
}

void TFBroadcaster::initSubscriber()
{
    test_sub_ = node_handle_.subscribe("/phantom/xyzrpy", 1, &TFBroadcaster::testCallback, this, ros::TransportHints().tcpNoDelay());
    joystick_pose_sub_ = node_handle_.subscribe("/dasom/EE_command", 1, &TFBroadcaster::joystickCallback, this, ros::TransportHints().tcpNoDelay());
    sub_EEpose_ = node_handle_.subscribe("/dasom/EE_pose", 10, &TFBroadcaster::PoseCallback, this); 
    sub_gimbal_tf_ = node_handle_.subscribe("/dasom/gimbal_tf", 10, &TFBroadcaster::gimbalCallback, this);
    gimbal_pose_sub_ = node_handle_.subscribe("/dasom/global_gimbal_pose", 1, &TFBroadcaster::globalgimbalCallback, this, ros::TransportHints().tcpNoDelay());                                              
    sub_optitrack_ = node_handle_.subscribe("/dasomPalletrone/world", 10, &TFBroadcaster::optitrackCallback, this, ros::TransportHints().tcpNoDelay());
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"tf_broadcaster");
    ros::NodeHandle nh;

    TFBroadcaster tf_br_;

    ros::Rate loop(100);

    tf_br_.time_i = ros::Time::now().toSec();
    tf_br_.time_f = 0;
    tf_br_.time_loop = 0;

    while(ros::ok())
    {
        tf_br_.time_f = ros::Time::now().toSec();
        tf_br_.time_loop = tf_br_.time_f - tf_br_.time_i;
        tf_br_.time_i = ros::Time::now().toSec();

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}