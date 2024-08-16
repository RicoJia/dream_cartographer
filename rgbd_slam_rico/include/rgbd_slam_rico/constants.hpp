#pragma once

#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "simple_robotics_cpp_utils/thread_pool.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>

namespace RgbdSlamRico {
constexpr const char *PACKAGE_NAME = "rgbd_slam_rico";
constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
constexpr auto DEPTH_TOPIC = "/camera/depth/image";
constexpr auto CAMERA_INFO_TOPIC = "/camera/rgb/camera_info";

// Initialized during runtime
const std::string FULL_PACKAGE_PATH = ros::package::getPath(PACKAGE_NAME);
const std::string PCD_FILE_NAME =
    FULL_PACKAGE_PATH + "/data/rgbd_rico_slam_output.pcd";

// Typedefs
// Define the IO format to print on one line
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Global variables
Eigen::IOFormat eigen_1_line_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                 ", ", ", ", "", "", " [", "] ");
ThreadPool point_cloud_addition_thread_pool(
    std::thread::hardware_concurrency() / 2); // using half of hardware cores

/************************************** Data Structures
 * **************************************/

struct SLAMParams {
  // Data params
  std::string bag_name = "";

  double max_depth = 20.0;
  double min_depth = 0.3;
  bool use_ransac_for_pnp = false;

  int min_ransac_feature_inliers =
      10; //> DLT algorithm needs at least 6 points for pose estimation from
  // 3D-2D point correspondences. (expected: 'count >= 6'), where
  double min_interframe_rotation_thre = 0.05;
  double min_interframe_translation_thre = 0.05;
  double max_interframe_rotation_thre = 1.57;
  double max_interframe_translation_thre = 1.0;
  bool downsample_point_cloud = false;
  float voxel_size = 0.01;
  int nearby_vertex_check_num = 0;
  int random_vertex_check_num = 0;
  std::string robust_kernel_name = "Cauchy";

  // Debugging params
  bool verbose = false;
  bool visualize_frames = false;
  bool do_ba_backend = true;
  bool pause_after_optimization = false;
  int initial_image_skip_num = 0;
  int image_skip_batch_num = 0;
  int image_num = 1;
  bool test_with_optimization = true;
  int pnp_method_enum = 0;

  bool save_pcd_file = false;
};

struct FrontEndData {
  cv::Mat image;
  cv::Mat depth_image;
  std::vector<cv::Point3f> object_frame_points;
  std::vector<cv::Point2f> current_camera_pixels;
};

struct KeyFrameData {
  cv::Mat image;
  cv::Mat depth_image;
  Eigen::Isometry3d pose;
  unsigned int index;
  ORBFeatureDetectionResult orb_res;
};

}; // namespace RgbdSlamRico
