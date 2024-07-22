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
  double max_depth = 20.0;
  double min_depth = 0.3;
  bool use_ransac_for_pnp = false;
  int min_matches_num =
      6; //> DLT algorithm needs at least 6 points for pose estimation from
         // 3D-2D point correspondences. (expected: 'count >= 6'), where
  double min_interframe_rotation_thre = 0.05;
  double min_interframe_translation_thre = 0.05;
  double max_interframe_rotation_thre = 1.57;
  double max_interframe_translation_thre = 1.0;
  bool downsample_point_cloud = false;
  float voxel_size = 0.01;

  // Debugging params
  bool verbose = false;
  bool do_ba_backend = true;
  bool pause_after_optimization = false;
  int initial_image_skip_num = 0;
  int image_skip_batch_num = 0;
  int image_num = 1;
  bool test_with_optimization = true;
  int pnp_method_enum = 0;
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
  if (orb_res.keypoints.size() < slam_params.min_matches_num) {
    std::cerr << "Not enough keypoints" << std::endl;
    return std::nullopt;
  }
  return orb_res;
}

/**
 * @brief filter out matches with invalid depths IN PLACE, i.e., depth outside
 * of [min_threshold, max_threshold]
 *
 * @param matches : point matches across two RGB images
 * @param depth1 : depth image of the first RGB image
 * @param depth2 : depth image of the second RGB image
 */
inline void filter_point_matches_with_valid_depths(
    std::vector<cv::DMatch> &matches, const cv::Mat &depth_img1,
    const cv::Mat &depth_img2, const ORBFeatureDetectionResult &res1,
    const ORBFeatureDetectionResult &res2, const SLAMParams &slam_params) {
  // NEED TO TEST??
  auto remove_invalid_matches = [&](const cv::DMatch &match) {
    auto p1 = res1.keypoints.at(match.queryIdx).pt;
    double depth1 = depth_img1.at<float>(int(p1.y), int(p1.x));
    if (std::isnan(depth1) || depth1 < slam_params.min_depth ||
        depth1 > slam_params.max_depth)
      return true;

    auto p2 = res2.keypoints.at(match.trainIdx).pt;
    double depth2 = depth_img2.at<float>(int(p2.y), int(p2.x));
    if (std::isnan(depth2) || depth2 < slam_params.min_depth ||
        depth2 > slam_params.max_depth)
      return true;

    return false;
  };
  auto new_end =
      std::remove_if(matches.begin(), matches.end(), remove_invalid_matches);
  matches.erase(new_end, matches.end());
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
          KeyFrameData &current_keyframe) {
  auto orb_res_optional = get_valid_orb_features(slam_params, current_keyframe);
  if (!orb_res_optional.has_value())
    return std::nullopt;
  current_keyframe.orb_res = orb_res_optional.value();
  // Step 6: Match features
  auto good_matches = find_matches_and_draw_them(
      previous_keyframe.orb_res, current_keyframe.orb_res, false);
  filter_point_matches_with_valid_depths(
      good_matches, previous_keyframe.depth_image, current_keyframe.depth_image,
      previous_keyframe.orb_res, current_keyframe.orb_res, slam_params);
  if (good_matches.size() < slam_params.min_matches_num) {
    std::cerr << "Not enough feature matches" << std::endl;
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
    // TODO
    std::cout << "obj size: " << object_frame_points.size() << std::endl;
    std::cout << "pixel size: " << current_camera_pixels.size() << std::endl;
    std::cout << "good match size: " << good_matches.size()
              << ", slam_params.min_matches_numL "
              << slam_params.min_matches_num << std::endl;
    try {
      // There's a bug where solvePnPRansac could have less than 6 points for
      // the DLT solver it uses. Related:
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
    return std::nullopt;
  }

  if (r_norm > slam_params.max_interframe_rotation_thre ||
      t_norm > slam_params.max_interframe_translation_thre) {
    std::cerr << "r and/or t norm is greater than motion thresholds. Skip."
              << std::endl;
    return std::nullopt;
  }

  cv::Mat R;
  cv::Rodrigues(r, R);
  Eigen::Isometry3d frame1_to_frame2 =
      SimpleRoboticsCppUtils::cv_R_t_to_eigen_isometry3d(R, t);
  return frame1_to_frame2;
}
} // namespace RgbdSlamRico
