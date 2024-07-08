#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <ros/ros.h>

using namespace RgbdSlamRico;
constexpr unsigned int IMAGE_SAMPLE_NUM = 5;

int main(int argc, char **argv) {
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;
  // This block to move to bag reader
  constexpr auto PACKAGE_NAME = "rgbd_slam_rico";

  SimpleRoboticsRosUtils::BagParser bp(nh, PACKAGE_NAME,
                                       "/data/rgbd_dataset_freiburg1_xyz.bag");
  constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
  bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);
  constexpr auto TF_TOPIC = "/camera/rgb/image_color";
  bp.add_topic<sensor_msgs::Image>(TF_TOPIC, false);
  constexpr auto CAMERA_INFO_TOPIC = "/camera/depth/camera_info";
  bp.add_topic<sensor_msgs::CameraInfo>(CAMERA_INFO_TOPIC, false);

  HandyCameraInfo cam_info = load_camera_info(bp, CAMERA_INFO_TOPIC);
  std::cout << "cam info: " << cam_info.K << std::endl;

  ORBFeatureDetectionResult last_orb_result;
  for (unsigned int i = 0; i < 5 * IMAGE_SAMPLE_NUM; i++) {
    auto image = load_next_image(bp, RGB_TOPIC);
    if (i % IMAGE_SAMPLE_NUM != 0)
      continue;

    auto orb_res = detect_orb_features(image);
    // TODO
    // display_image(orb_res.image_with_keypoints);
    if (!last_orb_result.is_null()) {
      auto [good_matches, img_with_matches] =
          find_matches_and_draw_them(last_orb_result, orb_res);
      SimpleRoboticsCppUtils::display_image(img_with_matches);

      // Part 2 SLAM Front end with 2D-2D Methods
      pose_estimate_2d2d(last_orb_result, orb_res, good_matches, cam_info.K);
    }
    last_orb_result = std::move(orb_res);
  }
}