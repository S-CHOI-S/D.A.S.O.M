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
double roll, pitch, yaw;
void TFBroadcaster::palletroneOptitrackCallback(const geometry_msgs::PoseStamped& msg)
// 기준: Z = 0.38
{
    // double roll, pitch, yaw;

    optitrackQuat[0] = msg.pose.position.x;
    optitrackQuat[1] = msg.pose.position.y;
    optitrackQuat[2] = msg.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    roll = roll - 0.009670;
    pitch = pitch - (-0.029053);
    yaw = yaw - (0.044478);

    quat.setRPY(roll, pitch, yaw);

    optitrackQuat[3] = quat.x();
    optitrackQuat[4] = quat.y();
    optitrackQuat[5] = quat.z();
    optitrackQuat[6] = quat.w();

    // ROS_ERROR("%lf, %lf, %lf, %lf, %lf, %lf", optitrackQuat[0], optitrackQuat[1], optitrackQuat[2], roll, pitch,yaw);

    // world2palletrone(optitrackQuat);
}

void TFBroadcaster::world2palletrone(Eigen::VectorXd optitrackquat, double r, double p, double y)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    double lpf_r, lpf_p, lpf_y;

    optitrackQuat_lpf[0] = ds_jnt_->updateLPF(time_loop, optitrackquat[0]);
    optitrackQuat_lpf[1] = ds_jnt_->updateLPF(time_loop, optitrackquat[1]);
    optitrackQuat_lpf[2] = ds_jnt_->updateLPF(time_loop, optitrackquat[2]);
    lpf_r = ds_jnt_->updateLPF(time_loop, r);
    lpf_p = ds_jnt_->updateLPF(time_loop, p);
    lpf_y = ds_jnt_->updateLPF(time_loop, y);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/palletrone_optitrack";
    transformStamped.transform.translation.x = optitrackQuat_lpf[0];
    transformStamped.transform.translation.y = optitrackQuat_lpf[1];
    transformStamped.transform.translation.z = optitrackQuat_lpf[2];

    tf::Quaternion quat;

    quat.setRPY(lpf_r, lpf_p, lpf_y);

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    br.sendTransform(transformStamped);
}


void TFBroadcaster::dasomEEPoseCallback(const geometry_msgs::Twist& msg)
{
    static tf2_ros::StaticTransformBroadcaster br_pose;
    geometry_msgs::TransformStamped transformStamped_pose;

    transformStamped_pose.header.stamp = ros::Time::now();
    transformStamped_pose.header.frame_id = "tf/palletrone_optitrack";
    transformStamped_pose.child_frame_id = "tf/global_EE_pose";
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

void TFBroadcaster::globalFixedGimbalPoseCallback(const geometry_msgs::PoseStamped &msg)
{  
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/global_fixed_gimbal_tf";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    br.sendTransform(transformStamped);

    ROS_INFO("DONE!");
    ROS_WARN("GIMBAL TF = %lf, %lf, %lf", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

void TFBroadcaster::initSubscriber()
{                                          
    palletrone_optitrack_sub_ = node_handle_.subscribe("/dasomPalletrone2/world", 10, &TFBroadcaster::palletroneOptitrackCallback, this, ros::TransportHints().tcpNoDelay());
    dasom_EE_pose_sub_ = node_handle_.subscribe("/dasom/EE_pose", 10, &TFBroadcaster::dasomEEPoseCallback, this); 
    global_fixed_gimbal_pose_sub_ = node_handle_.subscribe("/dasom/tf/global_fixed_gimbal_EE_pose", 10, &TFBroadcaster::globalFixedGimbalPoseCallback, this, ros::TransportHints().tcpNoDelay());
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

        tf_br_.world2palletrone(tf_br_.optitrackQuat, roll, pitch, yaw);

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}