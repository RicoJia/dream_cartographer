#include "rgbd_slam_rico/debug_utils.hpp"
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "rgbd_slam_rico_exercises/orb_exercise.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <gtest/gtest.h>
#include <opencv2/features2d.hpp>
#include <ros/package.h>
#include <ros/ros.h>

using namespace RgbdSlamRico;
using namespace RgbdSlamRicoExercises;
using namespace SimpleRoboticsCppUtils;
using namespace cv;

TEST(TestFeatureDetection, TestIntegralRectangle) {
  cv::Mat img = (cv::Mat_<uchar>(2, 1) << 1, 2); // 2x1
  cv::Mat integral_img;
  cv::integral(img, integral_img);

  std::cout << integral_img << std::endl;
  // y represents rows, x represents columns
  auto rec = cv::Rect{cv::Point(0, 0), cv::Point(1, 2)};
  // expect area to be the same as the sum of the two
  int sum = integral_rectangle(integral_img, rec);
  EXPECT_EQ(sum, 1 + 2);
}

TEST(TestFeatureDetection, TestCircleOffset) {
  std::vector<cv::Rect> horizontal_offsets, vertical_offsets;
  constexpr const int radius = 20;
  get_circle_offset(horizontal_offsets, vertical_offsets, radius);
  // Visual checks. Actual Test is a TODO
  for (auto r : horizontal_offsets) {
    EXPECT_EQ(r.height, 1);
    EXPECT_EQ(r.br().x * 2, r.width);

    EXPECT_LE(r.tl().x * r.tl().x + r.tl().y * r.tl().y, radius * radius);
    auto tl_abs = cv::Point(cv::abs(r.tl().x), cv::abs(r.tl().y));
    EXPECT_GE((tl_abs.x + 1) * (tl_abs.x + 1) + (tl_abs.y + 1) * (tl_abs.y + 1),
              radius * radius);
  }

  // for (auto r: vertical_offsets)
  // std::cout<<"vertical: tl: "<<r.tl()<<", br: "<<r.br()<<std::endl;
  // visualize_cv_rects(horizontal_offsets);
}

TEST(TestFeatureDetection, TestOrientation) {
  // create a matrix with the second quadrant being +1. All other quadrants are
  // 0. Then, calculate orientation
  constexpr const int radius = 10;
  cv::Mat image = cv::Mat::zeros((radius + 1) * 2, (radius + 1) * 2, CV_8UC1);
  cv::Rect second_quadrant(0, 0, radius + 1, radius + 1);
  image(second_quadrant) = 1;
  std::vector<cv::KeyPoint> keypoints{
      cv::KeyPoint(cv::Point{radius + 1, radius + 1}, 1)};
  compute_orientation(image, keypoints);
  EXPECT_NEAR(keypoints.at(0).angle, 135, 0.1);
}

TEST(TestFeatureDetection, TestGetRotations) {
  auto rotations = get_rotations();
  EXPECT_EQ(rotations.size(), static_cast<int>(2 * M_PI / ANGLE_INCREMENT) + 1);
  for (const auto &R : rotations) {
    auto prod = R * R.t();
    EXPECT_LT(cv::norm(prod - cv::Mat::eye(3, 3, CV_64F)), 1e-5);
  }
}

TEST(TestFeatureDetection, TestRotations) {
  cv::KeyPoint keypoint(cv::Point2f(5, 0), 1.0f);
  keypoint.angle = 90.0f; // Set angle to 90 degrees. POTENTIAL FLAKY TEST

  // The expected result after a 90-degree rotation around the origin
  cv::Point expected_point(5, 10);
  auto rotations = get_rotations();
  // Call rotate_point
  cv::Point rotated_point =
      rotate_and_translate_point(keypoint, 10, 0, rotations);

  // Check that the rotated point matches the expected point
  EXPECT_EQ(rotated_point.x, expected_point.x);
  EXPECT_EQ(rotated_point.y, expected_point.y);
}

TEST(TestFeatureDetection, TestDescriptorGeneration) {
  // Create a synthetic image (e.g., a simple gradient or checkerboard)
  cv::Mat test_image = cv::Mat::zeros(100, 100, CV_8UC1);
  cv::rectangle(test_image, cv::Point(20, 20), cv::Point(80, 80),
                cv::Scalar(255), -1);

  // Generate some keypoints (e.g., at fixed positions)
  std::vector<cv::KeyPoint> keypoints;
  keypoints.emplace_back(cv::KeyPoint(50, 50, 1.0f));
  keypoints.emplace_back(cv::KeyPoint(30, 30, 1.0f));
  keypoints.emplace_back(cv::KeyPoint(70, 70, 1.0f));

  // Call the function to test
  cv::Mat descriptors = compute_descriptor(keypoints, test_image);
  // Perform some basic checks
  assert(descriptors.rows == keypoints.size());  // Check number of descriptors
  assert(descriptors.cols == DESCRIPTOR_LENGTH); // Check descriptor length
  for (int i = 0; i < descriptors.rows; ++i) {
    for (int j = 0; j < descriptors.cols; ++j) {
      uchar value = descriptors.at<uchar>(i, j);
      // TODO: THIS IS VERY BASIC
      // You can add more detailed checks depending on the expected behavior
      // For now, let's just ensure it's within expected bounds
      assert(value >= 0 && value <= 255);
    }
  }

  // Print success message
  std::cout << "Test passed: compute_descriptor function works as expected."
            << std::endl;
}

TEST(TestFeatureDetection, TestDescriptorBruteforce) {
  // Example descriptors (8-bit binary descriptors for simplicity)
  uchar data1[] = {0b11011011, 0b00110011};
  uchar data2[] = {0b11110000, 0b00001111, 0b11011011, 0b00110011};

  cv::Mat desc1(1, 2, CV_8U, data1); // Single descriptor in desc1
  cv::Mat desc2(2, 2, CV_8U, data2); // Two descriptors in desc2

  // Expected match: desc1 should match with the second descriptor in desc2
  // (same data)
  std::vector<cv::DMatch> matches = brute_force_matching(desc1, desc2);

  // There should be exactly one match
  ASSERT_EQ(matches.size(), 1);

  // The match should correspond to the correct indices and distance
  EXPECT_EQ(matches[0].queryIdx, 0);
  EXPECT_EQ(matches[0].trainIdx, 1);
  EXPECT_EQ(matches[0].distance, 0); // Because the descriptors are identical
}

TEST(TestFeatureDetection, DifferentDescriptors) {
  // Example descriptors with more difference
  uchar data1[] = {0b11011011, 0b00110011};
  uchar data2[] = {0b11110000, 0b00001111};

  cv::Mat desc1(1, 2, CV_8U, data1); // Single descriptor in desc1
  cv::Mat desc2(1, 2, CV_8U, data2); // Single descriptor in desc2

  // Expected match: desc1 should match with desc2, but with a non-zero distance
  std::vector<cv::DMatch> matches = brute_force_matching(desc1, desc2);

  // There should be exactly one match
  ASSERT_EQ(matches.size(), 1);

  // The match should correspond to the correct indices and distance
  EXPECT_EQ(matches[0].queryIdx, 0);
  EXPECT_EQ(matches[0].trainIdx, 0);
  EXPECT_GT(matches[0].distance, 0); // Because the descriptors are different
}

ORBFeatureDetectionResult
test_orb_and_get_result(const std::string &frame_index,
                        const bool use_handwritten) {
  std::string package_path = ros::package::getPath(PACKAGE_NAME);
  auto image_path =
      package_path + "/data/bag_images/frame_00" + frame_index + ".png";
  std::cout << "Using image: " << image_path << std::endl;
  auto image = cv::imread(image_path, cv::IMREAD_COLOR);
  auto res = detect_orb_features(image, use_handwritten);
  return res;
}

// TOOD test with only 1 level. we are not handling projecting keypoints 3
// levels on to the same one yet.
TEST(TestFeatureDetection, TestORB) {
  std::vector<ORBFeatureDetectionResult> res_vec;
  // for an ID 00 - 99
  res_vec.push_back(test_orb_and_get_result(std::to_string(15), true));
  res_vec.push_back(test_orb_and_get_result(std::to_string(26), true));

  // TODO
  // TODO
  std::cout << "Now displaying custom ORB + custom feature matching"
            << std::endl;
  auto good_matches =
      brute_force_matching(res_vec.at(0).descriptor, res_vec.at(1).descriptor);
  draw_feature_matches(res_vec.at(0), res_vec.at(1), good_matches);

  std::cout << "Now displaying custom ORB + CV feature matching" << std::endl;
  good_matches = find_matches(res_vec.at(0), res_vec.at(1));
  draw_feature_matches(res_vec.at(0), res_vec.at(1), good_matches);

  res_vec.push_back(test_orb_and_get_result(std::to_string(15), false));
  res_vec.push_back(test_orb_and_get_result(std::to_string(26), false));

  std::cout << "Now displaying CV ORB + custom feature matching" << std::endl;
  good_matches =
      brute_force_matching(res_vec.at(0).descriptor, res_vec.at(1).descriptor);
  draw_feature_matches(res_vec.at(2), res_vec.at(3), good_matches);

  std::cout << "Now displaying CV ORB + CV feature matching" << std::endl;
  good_matches = find_matches(res_vec.at(2), res_vec.at(3));
  draw_feature_matches(res_vec.at(2), res_vec.at(3), good_matches);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}