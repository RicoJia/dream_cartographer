#pragma once

#include "simple_robotics_ros_utils/rosbag_helpers.hpp"
#include <Eigen/Dense>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tuple>
#include <vector>

namespace RgbdSlamRico {
namespace enc = sensor_msgs::image_encodings;

struct ORBFeatureDetectionResult {
  // keypoints.pt are stored as float values for subpixel accuracy.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptor;
  cv::Mat image_with_keypoints;
  cv::Mat image;
  bool is_null() const {
    return descriptor.empty() && image.empty() && image_with_keypoints.empty();
  }
};

struct HandyCameraInfo {
  Eigen::Matrix3d K;
};

inline cv::Mat load_rgbd_images(SimpleRoboticsRosUtils::BagParser &bp,
                                const std::string &image_topic_name) {
  auto msg = bp.next<sensor_msgs::Image>(image_topic_name);
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  return cv_ptr->image;
}

inline HandyCameraInfo load_camera_info(SimpleRoboticsRosUtils::BagParser &bp,
                                        const std::string &camera_info_topic) {

  auto msg = bp.next<sensor_msgs::CameraInfo>(camera_info_topic);
  HandyCameraInfo cam_info;
  cam_info.K =
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->K.data());
  return cam_info;
}

inline ORBFeatureDetectionResult detect_orb_features(cv::Mat &image) {
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  ORBFeatureDetectionResult res;
  res.image = image;

  orb->detectAndCompute(image, cv::noArray(), res.keypoints, res.descriptor);

  cv::drawKeypoints(image, res.keypoints, res.image_with_keypoints,
                    cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  return res;
}

inline std::tuple<std::vector<cv::DMatch>, cv::Mat>
find_matches_and_draw_them(const ORBFeatureDetectionResult &res1,
                           const ORBFeatureDetectionResult &res2) {
  cv::BFMatcher matcher;

  // Not using cv::DescriptorMatcher::FLANNBASED because we are doing binary
  // descriptors. FLANN is good for floating points
  std::vector<std::vector<cv::DMatch>> knn_matches;
  // find 2 neighbors, so we can do Lowe's ratio test: nearest neighbor/second
  // nearest neighbor
  matcher.knnMatch(res1.descriptor, res2.descriptor, knn_matches, 2);
  std::vector<cv::DMatch> good_matches;
  constexpr float lowe_ratio = 0.7f;
  for (const auto &knn_match_pair : knn_matches) {
    if (knn_match_pair[0].distance < lowe_ratio * knn_match_pair[1].distance)
      good_matches.emplace_back(knn_match_pair[0]);
  }
  cv::Mat image_with_matches;
  drawMatches(res1.image, res1.keypoints, res2.image, res2.keypoints,
              good_matches, image_with_matches, cv::Scalar::all(-1),
              cv::Scalar::all(-1), std::vector<char>(),
              cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  return std::make_tuple(good_matches, image_with_matches);
}

}; // namespace RgbdSlamRico