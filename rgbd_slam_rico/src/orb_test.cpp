#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include "simple_robotics_ros_utils/rosbag_helpers.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

using namespace RgbdSlamRico;
// void check_tf(){
//     // need to synchronize

// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "orb_test");
  ros::NodeHandle nh;
  // This block to move to bag reader
  constexpr auto PACKAGE_NAME = "rgbd_slam_rico";
  std::string package_path = ros::package::getPath(PACKAGE_NAME);
  if (package_path.empty())
    throw std::runtime_error(PACKAGE_NAME);
  auto bag_file_path =
      std::string(package_path) + "/data" + "/rgbd_dataset_freiburg1_xyz.bag";

  SimpleRoboticsRosUtils::BagParser bp(nh, bag_file_path);
  constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
  bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);
  constexpr auto TF_TOPIC = "/camera/rgb/image_color";
  bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);

  ORBFeatureDetectionResult last_orb_result;
  for (unsigned int i = 0; i < 5; i++) {
    auto image = load_rgbd_images(bp, RGB_TOPIC);
    auto orb_res = detect_orb_features(image);
    // TODO
    // display_image(orb_res.image_with_keypoints);
    if (!last_orb_result.is_null()) {
      auto [good_matches, img_matches] =
          find_matches_and_draw_them(last_orb_result, orb_res);
      SimpleRoboticsCppUtils::display_image(img_matches);
    }
    last_orb_result = std::move(orb_res);
  }
}