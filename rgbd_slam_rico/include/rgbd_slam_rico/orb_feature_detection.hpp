#pragma once

#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include "simple_robotics_ros_utils/rosbag_helpers.hpp"
#include "rgbd_slam_rico_exercises/orb_exercise.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tuple>
#include <vector>

namespace RgbdSlamRico {
namespace enc = sensor_msgs::image_encodings;

struct ORBFeatureDetectionResult {
  // keypoints.pt are stored as float values for subpixel accuracy.
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptor;
  cv::Mat image;
  bool is_null() const { return descriptor.empty() && image.empty(); }
};

/*
TODO: this function should be moved elsewhere, maybe in simple_ros_utils
*/
struct HandyCameraInfo {
  cv::Mat K;
};

/**
 * @brief Return the next image / depth image from the TUM data set
 *
 * @param bp : Bag Parser
 * @param image_topic_name : topic
 * @param is_rgb_image : choose false if it's a depth image
 * @return cv::Mat : image. (Or empty cv::Mat if this is not valid)
 */
inline cv::Mat load_next_image_TUM(SimpleRoboticsRosUtils::BagParser &bp,
                                   const std::string &image_topic_name,
                                   const bool &is_rgb_image) {
  // an RGB image is usually 8 bit unsigned int, with 3 channels, CV_8UC3?
  // A depth image is 16-bit monochrome images in
  // The RGB and depth images are already pre-registered (1-1 correspondences)
  // in primesense The depth images are scaled by a factor of 1 i.e., a pixel
  // value of 1 in the depth image corresponds to a distance of 1 meter from the
  // camera
  auto msg = bp.next<sensor_msgs::Image>(image_topic_name);
  if (msg == nullptr)
    return cv::Mat();
  cv_bridge::CvImageConstPtr cv_ptr;
  if (is_rgb_image)
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  else {
    cv_ptr = cv_bridge::toCvShare(msg, enc::TYPE_32FC1);
    cv_ptr->image /= 1;
  }
  // Because msg is a boost::shared_ptr, it will go out of scope after this call
  // So we make a copy of the image here.
  return cv_ptr->image.clone();
}

inline HandyCameraInfo load_camera_info(SimpleRoboticsRosUtils::BagParser &bp,
                                        const std::string &camera_info_topic) {

  auto msg = bp.next<sensor_msgs::CameraInfo>(camera_info_topic);
  HandyCameraInfo cam_info;
  // 64FC1 is 64 double precision, 1 channel
  cam_info.K = cv::Mat(3, 3, CV_64FC1);
  memcpy(cam_info.K.data, msg->K.data(), msg->K.size() * sizeof(double));
  return cam_info;
}

inline ORBFeatureDetectionResult detect_orb_features(const cv::Mat &image, const bool& use_handwritten = false) {
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  ORBFeatureDetectionResult res;
  res.image = image;

  if (!use_handwritten)
    orb->detectAndCompute(image, cv::noArray(), res.keypoints, res.descriptor);
  else {
    // handwritten_orb(image, res.keypoints, res.descriptor);
  }

  return res;
}

inline std::vector<cv::DMatch>
find_matches(const ORBFeatureDetectionResult &res1,
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
  return good_matches;
}

}; // namespace RgbdSlamRico