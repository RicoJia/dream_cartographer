#pragma once
#include <opencv2/calib3d.hpp>

namespace RgbdSlamRico {
constexpr double RANSAC_PROB_THRESHOLD = 0.99;
// Given pixel p1 in image 1, and essential matrix E, ideally, the
// epipolar line of the pixel of image 2 is Ep1, which corresponds to where its
// match could land. So if your match is this threshold away, we don't think
// it's valid
constexpr int RANSAC_MAX_DIST_TO_EPIPOLAR_LINE = 1.5;

/**
 * @brief Use 2D-2D methods to estimate R and t, through 8 point algorithms, and
 DLT homography.

 * @throws value
 * @param feature_matches
 */
inline void pose_estimate_2d2d(const ORBFeatureDetectionResult &res1,
                               const ORBFeatureDetectionResult &res2,
                               const std::vector<cv::DMatch> &feature_matches,
                               const cv::Mat &K) {
  // Find pixel points of the matches
  // The OpenCV official documentation is very vague about this. This is what's
  // happening: Given std::vector<cv::KeyPoint> keypoints each row contains
  // keypoint1, keypoint2 In DMatches: queryID is the index of keypoint1 in the
  // first image trainID is the index of keypoint2 in the second image. Note,
  // each point is stored as float values so it's subpixel accurate
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

  // Find fundamental matrix: just using the 8 equations and find a solution
  // that satisfies the epipolar constraint.
  cv::Mat fundamental_matrix =
      findFundamentalMat(points1, points2, cv::FM_8POINT);

  // convert pixel points to canonical plane: x = (px - cx)/fx
  // SVD -> get t1, t2, R1, R2. Open CV uses 5 point algorithm
  cv::Mat essential_matrix, R, t;
  essential_matrix = cv::findEssentialMat(points1, points2, K, cv::RANSAC,
                                          RANSAC_PROB_THRESHOLD,
                                          RANSAC_MAX_DIST_TO_EPIPOLAR_LINE);
  // Estimate R and t. This function does "cheirality check"
  // Which triangulates matched points, and chooses the R and t that yields
  // positive z
  cv::recoverPose(essential_matrix, points1, points2, K, R, t);
  std::cout << "F:" << std::endl << fundamental_matrix << std::endl;
  std::cout << "E:" << std::endl << essential_matrix << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "t: " << t << std::endl;
}

} // namespace RgbdSlamRico
