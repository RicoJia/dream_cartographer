#pragma once

#include "simple_robotics_cpp_utils/thread_pool.hpp"
#include <ros/package.h>

namespace RgbdSlamRico {
constexpr const char *PACKAGE_NAME = "rgbd_slam_rico";
constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
constexpr auto DEPTH_TOPIC = "/camera/depth/image";
constexpr auto CAMERA_INFO_TOPIC = "/camera/rgb/camera_info";

// Initialized during runtime
const std::string FULL_PACKAGE_PATH = ros::package::getPath(PACKAGE_NAME);
const std::string PCD_FILE_NAME =
    FULL_PACKAGE_PATH + "data/rgbd_rico_slam_output.pcd";

// Typedefs
// Define the IO format to print on one line
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Global variables
Eigen::IOFormat eigen_1_line_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                 ", ", ", ", "", "", " [", "] ");
ThreadPool point_cloud_addition_thread_pool(
    std::thread::hardware_concurrency() / 2); // using half of hardware cores
};                                            // namespace RgbdSlamRico