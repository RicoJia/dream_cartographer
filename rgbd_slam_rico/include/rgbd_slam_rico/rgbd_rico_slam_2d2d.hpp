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

struct PoseEstimate2D {
  cv::Mat F;
  cv::Mat E;
  cv::Mat R;
  cv::Mat t;
};

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

} // namespace RgbdSlamRico