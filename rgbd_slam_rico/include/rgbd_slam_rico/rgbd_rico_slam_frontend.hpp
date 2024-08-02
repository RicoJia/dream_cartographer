#pragma once
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include "simple_robotics_cpp_utils/io_utils.hpp"
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace RgbdSlamRico {

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

///////////////////////////////////////////////////////////////////////////
// Version 2.0
///////////////////////////////////////////////////////////////////////////
// RANSAC can be only used with p3p, ap3p, and epnp
inline int read_pnp_method(const std::string &method) {
  if (method == "epnp")
    return cv::SOLVEPNP_EPNP;
  else if (method == "p3p") // Need 4 points only
    return cv::SOLVEPNP_P3P;
  else if (method == "ap3p") // Need 4 points only
    return cv::SOLVEPNP_AP3P;
  else if (method == "dls")
    return cv::SOLVEPNP_DLS;
  else if (method == "ippe") // Need 4 points only
    return cv::SOLVEPNP_IPPE;
  else if (method == "ippe_square") // Need 4 points only
    return cv::SOLVEPNP_IPPE_SQUARE;
  else if (method == "iterative") // Need 4 points only
    return cv::SOLVEPNP_ITERATIVE;
  else
    throw std::runtime_error("Please select a valid pnp method");
}

std::optional<ORBFeatureDetectionResult>
get_valid_orb_features(const SLAMParams &slam_params,
                       const KeyFrameData &current_keyframe) {
  auto orb_res = detect_orb_features(current_keyframe.image);
  if (orb_res.keypoints.size() < slam_params.min_ransac_feature_inliers) {
    std::cerr << "Not enough keypoints. Detected features: "
              << orb_res.keypoints.size() << std::endl;
    return std::nullopt;
  }
  return orb_res;
}

/**
 * @brief filter out orb results with invalid depths IN PLACE, i.e., depth
 * outside of [min_threshold, max_threshold]
 *
 * @param depth_img : depth images
 * @param slam_params : slam params
 * @param res : feature detecion result
 * @return true if there are results with valid depths. Otherwise false
 */
bool filter_orb_result_with_valid_depths(const cv::Mat &depth_img,
                                         const SLAMParams &slam_params,
                                         ORBFeatureDetectionResult &res) {
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(res.keypoints.size());
  cv::Mat descriptor;
  for (unsigned int i = 0; i < res.keypoints.size(); ++i) {
    auto &p = res.keypoints.at(i).pt;
    double depth = depth_img.at<float>(int(p.y), int(p.x));
    if (!(std::isnan(depth) || depth < slam_params.min_depth ||
          depth > slam_params.max_depth)) {
      keypoints.emplace_back(res.keypoints.at(i));
      descriptor.push_back(res.descriptor.row(static_cast<int>(i)));
    }
  }

  if (keypoints.size() < slam_params.min_ransac_feature_inliers) {
    std::cerr << "Not enough features left after depth filtering. Number of "
                 "features left: "
              << keypoints.size() << std::endl;
    return false;
  }
  res.keypoints = std::move(keypoints);
  res.descriptor = std::move(descriptor);
  return true;
}

/**
 * @brief Get data ready for Motion Estimation using 3D-2D correspondence.
 * Important: We assume ALL 3D and 2D POINTS have been depth-filtered
 *
 * @param object_frame_points : output 3D matched points in the object (world)
 * frame
 * @param current_camera_points : output 2D matched pixel in the current camera
 * frame
 * @param res1 : feature detection result of the previous image
 * @param res2 : feature detection result of the current camera image
 * @param feature_matches : OpenCV feature matching
 * @param K : camera intrinsics
 * @param depth1 : depth image associated with the previous image.
 */
inline void
get_object_and_2d_points(std::vector<cv::Point3f> &object_frame_points,
                         std::vector<cv::Point2f> &current_camera_pixels,
                         const ORBFeatureDetectionResult &res1,
                         const ORBFeatureDetectionResult &res2,
                         const std::vector<cv::DMatch> &feature_matches,
                         const cv::Mat &K, const cv::Mat &depth1,
                         const SLAMParams &slam_params) {
  for (const cv::DMatch &match : feature_matches) {
    auto previous_pt = res1.keypoints.at(match.queryIdx).pt;
    double depth = depth1.at<float>(int(previous_pt.y), int(previous_pt.x));
    // to canonical form, then to depth
    auto p_canonical = SimpleRoboticsCppUtils::pixel2cam(previous_pt, K);
    object_frame_points.emplace_back(p_canonical.x * depth,
                                     p_canonical.y * depth, depth);
    current_camera_pixels.emplace_back(res2.keypoints.at(match.trainIdx).pt);
  }
}
/**
 * @brief : Calculate a pose estimate using front end solve pnp,
 * Then perform checks to determine if this frame is a key frame,
 * Caveats:
 *  - If two frames do not have enough point matches, then no frames are
 * yielded. You will have to rely on global optimization, which is a hit or miss
 * TODO
 */
inline std::optional<Eigen::Isometry3d>
front_end(const HandyCameraInfo &cam_info, const SLAMParams &slam_params,
          const KeyFrameData &previous_keyframe,
          const KeyFrameData &current_keyframe) {
  // Step 6: Match features
  auto good_matches = find_matches_and_draw_them(previous_keyframe.orb_res,
                                                 current_keyframe.orb_res,
                                                 slam_params.visualize_frames);
  if (good_matches.size() < slam_params.min_ransac_feature_inliers) {
    std::cerr << "Not enough feature matches. Detected matches: "
              << good_matches.size() << std::endl;
    return std::nullopt;
  }
  // Step 1 - PnP front end
  const cv::Mat &depth1 = previous_keyframe.depth_image;
  const cv::Mat &K = cam_info.K;

  std::vector<cv::Point3f> object_frame_points;
  std::vector<cv::Point2f> current_camera_pixels;
  get_object_and_2d_points(object_frame_points, current_camera_pixels,
                           previous_keyframe.orb_res, current_keyframe.orb_res,
                           good_matches, K, depth1, slam_params);

  cv::Mat r, t;
  if (slam_params.use_ransac_for_pnp) {
    try {
      // There's a bug where solvePnPRansac could have less than 6 points for
      // the DLT solver it uses in solvePnP. Related:
      // https://github.com/opencv/opencv/pull/19253#discussion_r570670642
      cv::solvePnPRansac(object_frame_points, current_camera_pixels, K,
                         cv::Mat(), r, t, false, slam_params.pnp_method_enum);
    } catch (const cv::Exception &e) {
      std::cerr << SimpleRoboticsCppUtils::to_color_msg(
                       SimpleRoboticsCppUtils::ANSIStrings::YELLOW, e.what(),
                       true)
                << std::endl
                << SimpleRoboticsCppUtils::to_color_msg(
                       SimpleRoboticsCppUtils::ANSIStrings::BLUE,
                       "Using Regular SolvePnP", true)
                << std::endl;
      cv::solvePnP(object_frame_points, current_camera_pixels, K, cv::Mat(), r,
                   t, false, cv::SOLVEPNP_DLS);
    }

  } else {
    cv::solvePnP(object_frame_points, current_camera_pixels, K, cv::Mat(), r, t,
                 false, slam_params.pnp_method_enum);
  }

  // Step 2: checks:
  const double r_norm = cv::norm(r);
  const double t_norm = cv::norm(t);
  if (r_norm < slam_params.min_interframe_rotation_thre &&
      t_norm < slam_params.min_interframe_translation_thre) {
    std::cerr << "r and t norm are smaller than motion thresholds. Skip."
              << std::endl;
    std::cerr << "r_norm: " << r_norm << "r: " << r << std::endl;
    std::cerr << "t_norm: " << t_norm << "t: " << t << std::endl;
    return std::nullopt;
  }

  if (r_norm > slam_params.max_interframe_rotation_thre ||
      t_norm > slam_params.max_interframe_translation_thre) {
    std::cerr << "r and/or t norm is greater than motion thresholds. Skip."
              << std::endl;
    std::cerr << "r_norm: " << r_norm << "r: " << r << std::endl;
    std::cerr << "t_norm: " << t_norm << "t: " << t << std::endl;
    return std::nullopt;
  }

  cv::Mat R;
  cv::Rodrigues(r, R);
  Eigen::Isometry3d frame1_to_frame2 =
      SimpleRoboticsCppUtils::cv_R_t_to_eigen_isometry3d(R, t);
  return frame1_to_frame2;
}
} // namespace RgbdSlamRico
