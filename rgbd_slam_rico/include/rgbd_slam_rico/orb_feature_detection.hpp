#pragma once

#include <opencv2/features2d.hpp>
#include <tuple>
#include <vector>

namespace RgbdSlamRico {
namespace enc = sensor_msgs::image_encodings;

struct ORBFeatureDetectionResult {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptor;
  cv::Mat image_with_keypoints;
  cv::Mat image;
  bool is_null() const {
    return descriptor.empty() && image.empty() && image_with_keypoints.empty();
  }
};

cv::Mat load_rgbd_images(SimpleRoboticsRosUtils::BagParser &bp,
                         const std::string &image_topic_name) {
  auto msg = bp.next<sensor_msgs::Image>(image_topic_name);
  // TODO
  std::cout << "msg->image" << msg->height << std::endl;
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  return cv_ptr->image;
}

ORBFeatureDetectionResult detect_orb_features(cv::Mat &image) {
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  ORBFeatureDetectionResult res;
  res.image = image;

  orb->detectAndCompute(image, cv::noArray(), res.keypoints, res.descriptor);

  cv::drawKeypoints(image, res.keypoints, res.image_with_keypoints,
                    cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  return res;
}

std::tuple<std::vector<cv::DMatch>, cv::Mat>
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
  cv::Mat img_matches;
  drawMatches(res1.image, res1.keypoints, res2.image, res2.keypoints,
              good_matches, img_matches, cv::Scalar::all(-1),
              cv::Scalar::all(-1), std::vector<char>(),
              cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  return std::make_tuple(good_matches, img_matches);
}

}; // namespace RgbdSlamRico