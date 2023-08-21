#include <librealsense2/rs.hpp>
#include <iostream>

int main() {
    // Create a RealSense pipeline and configuration
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(rs2_stream::RS2_STREAM_DEPTH); // Enable depth stream

    // Start streaming
    pipeline.start(config);

    try {
        while (true) {
            // Wait for frames
            rs2::frameset frames = pipeline.wait_for_frames();
            rs2::depth_frame depth_frame = frames.get_depth_frame(); // Get depth frame

            // Get the depth value at a specific pixel (x, y)
            int x = 320; // Example pixel coordinates
            int y = 240;
            float depth_meters = depth_frame.get_distance(x, y); // Get depth in meters

            // Print the distance in meters
            std::cout << "Distance at (" << x << ", " << y << "): " << depth_meters << " meters" << std::endl;
        }
    } catch (const rs2::error &e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    }

    return 0;
}

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <librealsense2/rs.hpp>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "realsense_pointcloud_publisher");
//     ros::NodeHandle nh;

//     // Create a RealSense pipeline and configuration
//     rs2::pipeline pipeline;
//     rs2::config config;
//     config.enable_stream(rs2_stream::RS2_STREAM_DEPTH); // Enable depth stream

//     // Start streaming
//     pipeline.start(config);

//     // ROS Publisher for PointCloud2
//     ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("realsense_pointcloud", 1);

//     try {
//         while (ros::ok()) {
//             // Wait for frames
//             rs2::frameset frames = pipeline.wait_for_frames();
//             rs2::depth_frame depth_frame = frames.get_depth_frame(); // Get depth frame

//             // Convert depth frame to PointCloud2
//             sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
//             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//             for (int y = 0; y < depth_frame.get_height(); y++) {
//                 for (int x = 0; x < depth_frame.get_width(); x++) {
//                     pcl::PointXYZ point;
//                     float depth_meters = depth_frame.get_distance(x, y);
//                     point.x = depth_meters * (x - depth_frame.get_width() / 2) * 0.001;
//                     point.y = depth_meters * (y - depth_frame.get_height() / 2) * 0.001;
//                     point.z = depth_meters;
//                     cloud->push_back(point);
//                 }
//             }
//             pcl::toROSMsg(*cloud, *cloud_msg);
//             cloud_msg->header.frame_id = "camera_depth_optical_frame"; // Update frame_id
//             cloud_msg->header.stamp = ros::Time::now();
//             pointcloud_pub.publish(cloud_msg);

//             ros::spinOnce();
//         }
//     } catch (const rs2::error &e) {
//         ROS_ERROR_STREAM("RealSense error: " << e.what());
//     }

//     return 0;
// }
