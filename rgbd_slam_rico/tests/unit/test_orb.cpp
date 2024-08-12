#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include "rgbd_slam_rico_exercises/orb_exercise.hpp"
#include "rgbd_slam_rico/constants.hpp"
#include <opencv2/features2d.hpp>

using namespace RgbdSlamRico;
using namespace cv;

TEST(TestFeatureDetection, TestORB){
    std::string package_path = ros::package::getPath(PACKAGE_NAME);
    auto image_path = package_path + "/data/bag_images/frame_0000.png";
    std::cout<<"Using image: "<<image_path<<std::endl;
    auto image = cv::imread(image_path, cv::IMREAD_COLOR);
    std::vector<KeyPoint> keypoints;
    cv::Mat descriptors;
    RgbdSlamRicoExercises::handwritten_orb(image, keypoints, descriptors);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}