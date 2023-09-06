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

void TFBroadcaster::joystickCallback(const geometry_msgs::Twist& msg)
{
    static tf2_ros::StaticTransformBroadcaster br_pose;
    geometry_msgs::TransformStamped transformStamped_pose;

    transformStamped_pose.header.stamp = ros::Time::now();
    transformStamped_pose.header.frame_id = "world";
    transformStamped_pose.child_frame_id = "joystick_command";
    transformStamped_pose.transform.translation.x = msg.linear.x;
    transformStamped_pose.transform.translation.y = msg.linear.y;
    transformStamped_pose.transform.translation.z = msg.linear.z;

    tf::Quaternion quat;
    quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

    transformStamped_pose.transform.rotation.x = quat.x();
    transformStamped_pose.transform.rotation.y = quat.y();
    transformStamped_pose.transform.rotation.z = quat.z();
    transformStamped_pose.transform.rotation.w = quat.w();
    
    br_pose.sendTransform(transformStamped_pose);
}

void TFBroadcaster::world2palletrone(Eigen::VectorXd optitrackquat)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    optitrackQuat_lpf[0] = ds_jnt_->updateLPF(time_loop, optitrackquat[0]);
    optitrackQuat_lpf[1] = ds_jnt_->updateLPF(time_loop, optitrackquat[1]);
    optitrackQuat_lpf[2] = ds_jnt_->updateLPF(time_loop, optitrackquat[2]);
    optitrackQuat_lpf[3] = ds_jnt_->updateLPF(time_loop, optitrackquat[3]);
    optitrackQuat_lpf[4] = ds_jnt_->updateLPF(time_loop, optitrackquat[4]);
    optitrackQuat_lpf[5] = ds_jnt_->updateLPF(time_loop, optitrackquat[5]);
    optitrackQuat_lpf[6] = ds_jnt_->updateLPF(time_loop, optitrackquat[6]);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "paletrone";
    transformStamped.transform.translation.x = optitrackQuat_lpf[0];
    transformStamped.transform.translation.y = optitrackQuat_lpf[1];
    transformStamped.transform.translation.z = optitrackQuat_lpf[2];

    transformStamped.transform.rotation.x = optitrackQuat_lpf[3];
    transformStamped.transform.rotation.y = optitrackQuat_lpf[4];
    transformStamped.transform.rotation.z = optitrackQuat_lpf[5];
    transformStamped.transform.rotation.w = optitrackQuat_lpf[6];

    br.sendTransform(transformStamped);
}

void TFBroadcaster::optitrackCallback(const geometry_msgs::PoseStamped& msg)
{
    double roll = 5;
    double pitch;
    double yaw;

    optitrackQuat[0] = msg.pose.position.x - 0.083278;
    optitrackQuat[1] = msg.pose.position.y - 0.053549;
    optitrackQuat[2] = msg.pose.position.z - 0.001452;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // roll -= -0.006381 + 0.708853 -0.576069 -1.226498;
    // pitch -= 0.02034 + 0.048070 + 0.401560 -2.031406;
    // yaw -= 1.420537 + 0.702262 -0.559468 -2.055506;

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // roll -= -1.035882;
    // pitch -= -1.558368; 
    // yaw -= -0.560295;

    quat.setRPY(roll, pitch, yaw);

    optitrackQuat[3] = quat.x();
    optitrackQuat[4] = quat.y();
    optitrackQuat[5] = quat.z();
    optitrackQuat[6] = quat.w();

    ROS_ERROR("%lf, %lf, %lf, %lf, %lf, %lf", optitrackQuat[0], optitrackQuat[1], optitrackQuat[2], roll, pitch,yaw);

    world2palletrone(optitrackQuat);
}

void TFBroadcaster::PoseCallback(const geometry_msgs::Twist& msg)
{
    static tf2_ros::StaticTransformBroadcaster br_pose;
    geometry_msgs::TransformStamped transformStamped_pose;

    transformStamped_pose.header.stamp = ros::Time::now();
    transformStamped_pose.header.frame_id = "paletrone";
    transformStamped_pose.child_frame_id = "global_EE_pose";
    transformStamped_pose.transform.translation.x = msg.linear.x;
    transformStamped_pose.transform.translation.y = msg.linear.y;
    transformStamped_pose.transform.translation.z = msg.linear.z;
    
    tf::Quaternion quat;

    quat.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);

    transformStamped_pose.transform.rotation.x = quat.x();
    transformStamped_pose.transform.rotation.y = quat.y();
    transformStamped_pose.transform.rotation.z = quat.z();
    transformStamped_pose.transform.rotation.w = quat.w();
    
    br_pose.sendTransform(transformStamped_pose);
}

void TFBroadcaster::globalgimbalCallback(const geometry_msgs::PoseStamped &msg)
{  
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "global_gimbal_tf";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    br.sendTransform(transformStamped);
}

void TFBroadcaster::gimbalCallback(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br_gimbal;
    geometry_msgs::TransformStamped transformStamped_pose;

    transformStamped_pose.header.stamp = ros::Time::now();
    transformStamped_pose.header.frame_id = "paletrone";
    transformStamped_pose.child_frame_id = "gimbal_tf";
    transformStamped_pose.transform.translation.x = msg.pose.position.x;
    transformStamped_pose.transform.translation.y = msg.pose.position.y;
    transformStamped_pose.transform.translation.z = msg.pose.position.z;

    transformStamped_pose.transform.rotation.x = msg.pose.orientation.x;
    transformStamped_pose.transform.rotation.y = msg.pose.orientation.y;
    transformStamped_pose.transform.rotation.z = msg.pose.orientation.z;
    transformStamped_pose.transform.rotation.w = msg.pose.orientation.w;
    br_gimbal.sendTransform(transformStamped_pose);
}

void TFBroadcaster::initSubscriber()
{
    joystick_pose_sub_ = node_handle_.subscribe("/dasom/EE_command", 1, &TFBroadcaster::joystickCallback, this, ros::TransportHints().tcpNoDelay());
    sub_EEpose_ = node_handle_.subscribe("/dasom/EE_pose", 10, &TFBroadcaster::PoseCallback, this); 
    sub_gimbal_tf_ = node_handle_.subscribe("/dasom/gimbal_tf", 10, &TFBroadcaster::gimbalCallback, this);
    gimbal_pose_sub_ = node_handle_.subscribe("/dasom/global_gimbal_pose", 1, &TFBroadcaster::globalgimbalCallback, this, ros::TransportHints().tcpNoDelay());                                              
    sub_optitrack_ = node_handle_.subscribe("/dasombasePlate/world", 10, &TFBroadcaster::optitrackCallback, this, ros::TransportHints().tcpNoDelay());
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