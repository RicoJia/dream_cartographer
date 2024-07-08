#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>

using namespace RgbdSlamRico;
constexpr unsigned int IMAGE_SAMPLE_NUM = 5;
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rgbd_rico_slam_mini");
  ros::NodeHandle nh;

  // This block to move to bag reader
  constexpr auto PACKAGE_NAME = "rgbd_slam_rico";

  SimpleRoboticsRosUtils::BagParser bp(nh, PACKAGE_NAME,
                                       "/data/rgbd_dataset_freiburg1_xyz.bag");
  constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
  bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);
  constexpr auto DEPTH_TOPIC = "/camera/depth/image";
  bp.add_topic<sensor_msgs::Image>(DEPTH_TOPIC, false);

  for (unsigned int i = 0; i < 5 * IMAGE_SAMPLE_NUM; i++) {
    auto image = load_next_image(bp, RGB_TOPIC);
    auto depth_image = load_next_image(bp, DEPTH_TOPIC);
  }

  return 0;
}
