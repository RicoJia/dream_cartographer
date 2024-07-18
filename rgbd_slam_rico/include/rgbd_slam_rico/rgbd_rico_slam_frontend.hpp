#pragma once
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>

namespace RgbdSlamRico {

/************************************** Constants
 * **************************************/
constexpr double RANSAC_PROB_THRESHOLD = 0.99;
// Given pixel p1 in image 1, and essential matrix E, ideally, the
// epipolar line of the pixel of image 2 is Ep1, which corresponds to where its
// match could land. So if your match is this threshold away, we don't think
// it's valid
constexpr int RANSAC_MAX_DIST_TO_EPIPOLAR_LINE = 1.5;

/************************************** Data Structures
 * **************************************/
struct PoseEstimate2D {
  cv::Mat F;
  cv::Mat E;
  cv::Mat R;
  cv::Mat t;
};

struct PoseEstimate3D {
  cv::Mat R;
  cv::Mat t;
  std::vector<cv::Point3f> object_frame_points;
  std::vector<cv::Point2f> current_camera_pixels;
};

struct SLAMParams{
    double max_depth = 20.0;
    double min_depth = 0.3;
};

/************************************** Functions
 * **************************************/

inline int read_pnp_method(const std::string &method) {
  if (method == "epnp")
    return cv::SOLVEPNP_EPNP;
  else if (method == "p3p")
    return cv::SOLVEPNP_P3P;
  else if (method == "dls")
    return cv::SOLVEPNP_DLS;
  else
    return cv::SOLVEPNP_EPNP;
}

inline double compare_matrices_to_scale(const cv::Mat &m1, const cv::Mat &m2) {
  double scale_factor = SimpleRoboticsCppUtils::getFrobeniusNorm(m1) /
                        SimpleRoboticsCppUtils::getFrobeniusNorm(m2);
  auto m2_normalized = m2 * scale_factor;
  double difference = SimpleRoboticsCppUtils::getFrobeniusNorm(
      cv::abs(m1) - cv::abs(m2_normalized));
  return difference;
}

inline void pose_estimate_2d_2d_check(const PoseEstimate2D &pe,
                                      const cv::Mat &K,
                                      std::vector<cv::Point2f> points1,
                                      std::vector<cv::Point2f> points2) {
  std::cout << "F:" << std::endl << pe.F << std::endl;
  std::cout << "E:" << std::endl << pe.E << std::endl;
  std::cout << "R: " << pe.R << std::endl;
  std::cout << "t: " << pe.t << std::endl;

  // E = txR. note: this E_posterior should multiply a scale factor
  // The scale factor is actually negative, because that doesn't change its
  // validity in epipolar constraint.
  cv::Mat E_posterior = SimpleRoboticsCppUtils::skew_symmetric_mat(pe.t) * pe.R;
  double difference = compare_matrices_to_scale(pe.E, E_posterior);
  std::cout << "Total Forbenius Difference: " << difference << std::endl;
  // - foundamental matrix = K^-T E K^-1
  cv::Mat K_inv;
  if (SimpleRoboticsCppUtils::invert_mat(K, K_inv)) {
    auto F_posterior = K_inv.t() * E_posterior * K_inv;
    double f_diff = compare_matrices_to_scale(F_posterior, pe.F);
    std::cout << "f_diff: " << f_diff << std::endl;

  } else {
    std::cout << "K_inv is not valid" << std::endl;
  }
  // Reproject points from canonical plane 1 to canonical plane 2.
  // - Epipolar Constraints: get canonical plane coords, Put them in P E P
  // Ep_c1 will be the parameters of the epipolar line of image 2
  std::cout << "E_posterior: " << E_posterior << std::endl;
  auto canonical_points1 = SimpleRoboticsCppUtils::pixel2cam(points1, K);
  auto canonical_points2 = SimpleRoboticsCppUtils::pixel2cam(points2, K);
  std::vector<double> distancias;
  distancias.reserve(canonical_points1.size());
  for (unsigned int i = 0; i < canonical_points1.size(); ++i) {
    const auto &p_c1 = canonical_points1[i];
    const auto &p_c2 = canonical_points2[i];
    cv::Mat p1 = (cv::Mat_<double>(3, 1) << p_c1.x, p_c1.y, 1.0);
    cv::Mat l2 = pe.E * p1;
    const double a = l2.at<double>(0);
    const double b = l2.at<double>(1);
    const double c = l2.at<double>(2);
    distancias.emplace_back(std::abs(a * p_c2.x + b * p_c2.y + c) /
                            std::sqrt(a * a + b * b));
    std::cout << distancias.back() << std::endl;
  }
  double sum = std::accumulate(distancias.begin(), distancias.end(), 0.0);
  std::cout << "Average reprojection error: "
            << sum / (static_cast<double>(distancias.size())) << std::endl;
};

/**
 * @brief Use 2D-2D methods to estimate R and t, through 8 point algorithms, and
 DLT homography.
 * @throws runtime error if less than 8 match pairs are found.
 * @param feature_matches
 */
inline PoseEstimate2D
pose_estimate_2d2d(const ORBFeatureDetectionResult &res1,
                   const ORBFeatureDetectionResult &res2,
                   const std::vector<cv::DMatch> &feature_matches,
                   const cv::Mat &K) {
  /**
    Find pixel points of the matches
    The OpenCV official documentation is very vague about this. This is what's
    happening: Given std::vector<cv::KeyPoint> keypoints each row contains
    keypoint1, keypoint2 In DMatches: queryID is the index of keypoint1 in the
    first image trainID is the index of keypoint2 in the second image. Note,
    each point is stored as float values so it's subpixel accurate
   */
  unsigned int pixel_count = feature_matches.size();
  if (pixel_count < 8)
    throw std::runtime_error(
        "Number of matched points must be greater than 8.");
  std::vector<cv::Point2f> points1, points2;
  points1.reserve(pixel_count);
  points2.reserve(pixel_count);
  for (const auto &match : feature_matches) {
    points1.emplace_back(res1.keypoints.at(match.queryIdx).pt);
    points2.emplace_back(res2.keypoints.at(match.trainIdx).pt);
  }

  /**
    Find fundamental matrix: just using the 8 equations and find a solution
    that satisfies the epipolar constraint.
   */
  cv::Mat fundamental_matrix =
      findFundamentalMat(points1, points2, cv::FM_8POINT);

  /**
    Internally, convert pixel points to canonical plane: x = (px - cx)/fx
    SVD -> get t1, t2, R1, R2. Open CV uses 5 point algorithm
    Remember, E = txR, where R and t transforms points1 (canonical) to points2
    (canonical).
   */
  cv::Mat essential_matrix, R, t;
  essential_matrix = cv::findEssentialMat(points1, points2, K, cv::RANSAC,
                                          RANSAC_PROB_THRESHOLD,
                                          RANSAC_MAX_DIST_TO_EPIPOLAR_LINE);
  /**
    Estimate R and t. This function does "cheirality check"
    Which triangulates matched points, and chooses the R and t that yields
    positive z
   */
  cv::recoverPose(essential_matrix, points1, points2, K, R, t);

  PoseEstimate2D pe{fundamental_matrix, essential_matrix, R, t};
  pose_estimate_2d_2d_check(pe, K, points1, points2);
  return pe;
}

/**
 * @brief Get data ready for Motion Estimation using 3D-2D correspondence
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
                         const SLAMParams& slam_params) {
  for (const cv::DMatch &match : feature_matches) {
    auto previous_pt = res1.keypoints.at(match.queryIdx).pt;
    double depth = depth1.at<float>(int(previous_pt.y), int(previous_pt.x));
    if (std::isnan(depth) || depth < slam_params.min_depth || depth > slam_params.max_depth)
      continue; // bad depth
    // to canonical form, then to depth
    auto p_canonical = SimpleRoboticsCppUtils::pixel2cam(previous_pt, K);
    object_frame_points.emplace_back(p_canonical.x * depth,
                                     p_canonical.y * depth, depth);
    current_camera_pixels.emplace_back(res2.keypoints.at(match.trainIdx).pt);
  }
}
inline PoseEstimate3D
pose_estimate_3d2d_opencv(const ORBFeatureDetectionResult &res1,
                          const ORBFeatureDetectionResult &res2,
                          const std::vector<cv::DMatch> &feature_matches,
                          const cv::Mat &K, const cv::Mat &depth1,
                          const int &pnp_method_enum, const SLAMParams& slam_params) {
  std::vector<cv::Point3f> object_frame_points;
  std::vector<cv::Point2f> current_camera_pixels;
  get_object_and_2d_points(object_frame_points, current_camera_pixels, res1,
                           res2, feature_matches, K, depth1, slam_params);

  // for(auto p: object_frame_points) std::cout<<"3d p: "<<p<<std::endl;
  cv::Mat r, t;

  /**
  cv::SOLVEPNP_DLS: "A Direct Least-Squares (DLS) Method for PnP"
  cv::SOLVEPNP_P3P : "Complete Solution Classification for the
  Perspective-Three-Point Problem". It needs 4 points exactly cv::SOLVEPNP_EPNP
  : "Complete Solution Classification for the Perspective-Three-Point Problem".
  It needs 4 points exactly cv::Mat() is distCoeffs
   */
  // TODO: to add our custom patches.
  cv::solvePnP(object_frame_points, current_camera_pixels, K, cv::Mat(), r, t,
               false, pnp_method_enum);
  cv::Mat R;
  cv::Rodrigues(r, R);

  return {R, t, std::move(object_frame_points),
          std::move(current_camera_pixels)};
}

} // namespace RgbdSlamRico
