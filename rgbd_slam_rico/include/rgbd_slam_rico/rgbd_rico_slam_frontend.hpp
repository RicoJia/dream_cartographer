#pragma once
#include <opencv2/calib3d.hpp>

namespace RgbdSlamRico {
//
//
// inputs: point matches, Intrinsics (fx, fy, c1, c2)
/**
 * @brief Use 2D-2D methods to estimate R and t, through 8 point algorithms, and
 DLT homography.

 * @throws value
 * @param feature_matches
 */
inline void pose_estimate_2d2d(const ORBFeatureDetectionResult &res1,
                               const ORBFeatureDetectionResult &res2,
                               const std::vector<cv::DMatch> &feature_matches) {
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
    // TODO
    // std::cout<<"points1: "<<res1.keypoints.at(match.queryIdx).pt<<std::endl;
  }

  // Find fundamental matrix: just using the 8 equations and find a solution
  // that satisfies the epipolar constraint.
  cv::Mat fundamental_matrix =
      findFundamentalMat(points1, points2, cv::FM_8POINT);

  // TODO
  std::cout << "F:" << std::endl << fundamental_matrix << std::endl;

  // convert pixel points to canonical plane
  // Estimate R and t
  // SVD -> get t1, t2, R1, R2.
  // Plug canonical point in? Get positive z.
}

} // namespace RgbdSlamRico
