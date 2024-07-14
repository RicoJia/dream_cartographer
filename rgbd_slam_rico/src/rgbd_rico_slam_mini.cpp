#include "rgbd_slam_rico/rgbd_rico_slam_backend.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace RgbdSlamRico;
constexpr unsigned int IMAGE_SAMPLE_NUM = 5;

// TODO: move to simple robotics utils
void visualize_slam_results(
    const std::vector<Eigen::Isometry3d> &optimized_poses,
    ros::Publisher &poses_publisher, ros::Publisher &point_cloud_publisher,
    PointCloud::Ptr point_cloud) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "map";

  for (const auto &pose : optimized_poses) {
    geometry_msgs::Pose p;
    p.position.x = pose.translation().x();
    p.position.y = pose.translation().y();
    p.position.z = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    pose_array.poses.push_back(p);
  }
  poses_publisher.publish(pose_array);
  point_cloud_publisher.publish(point_cloud);
}

void add_point_cloud(const Eigen::Isometry3d &pose, const cv::Mat &image,
                     const cv::Mat &depth_image,
                     const HandyCameraInfo &cam_info,
                     PointCloud::Ptr point_cloud) {

  for (unsigned int v = 0; v < depth_image.rows; ++v) {
    for (unsigned int u = 0; u < depth_image.rows; ++u) {
      // step 1: get depth
      double depth = depth_image.at<float>(v, u);
      if (std::isnan(depth) || depth < MIN_DEPTH || depth > MAX_DEPTH)
        continue; // bad depth

      // step 2: get canonical coords, and 3D point
      // TODO: u,v here
      auto p_canonical = SimpleRoboticsCppUtils::pixel2cam({u, v}, cam_info.K);
      Eigen::Vector3d point{p_canonical.x * depth, p_canonical.y * depth,
                            depth};
      auto world_point = pose * point;
      //  Is it bgr or rgb?
      cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);
      pcl::PointXYZRGB pcl_point;
      pcl_point.x = world_point[0];
      pcl_point.y = world_point[1];
      pcl_point.z = world_point[2];
      pcl_point.r = world_point[2];
      pcl_point.g = world_point[1];
      pcl_point.b = world_point[0];
      point_cloud->points.push_back(pcl_point);
    }
  }
  point_cloud->width = point_cloud->points.size();
  // still say is not dense so there might be invalid points
  point_cloud->is_dense = false;
}

int main(int argc, char *argv[]) {
  // Step 1: set up ros node, bagparser, topics and camera info
  ros::init(argc, argv, "rgbd_rico_slam_mini");
  ros::NodeHandle nh;
  ros::Publisher poses_pub =
      nh.advertise<geometry_msgs::PoseArray>("optimized_poses", 1, true);
  ros::Publisher points_pub =
      nh.advertise<PointCloud>("optimized_points", 1, true);
  constexpr auto PACKAGE_NAME = "rgbd_slam_rico";
  SimpleRoboticsRosUtils::BagParser bp(nh, PACKAGE_NAME,
                                       "/data/rgbd_dataset_freiburg1_xyz.bag");
  constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
  bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);
  constexpr auto DEPTH_TOPIC = "/camera/depth/image";
  bp.add_topic<sensor_msgs::Image>(DEPTH_TOPIC, false);
  constexpr auto CAMERA_INFO_TOPIC = "/camera/rgb/camera_info";
  HandyCameraInfo cam_info = load_camera_info(bp, CAMERA_INFO_TOPIC);
  ORBFeatureDetectionResult last_orb_result;

  PointCloud::Ptr point_cloud(new PointCloud);
  point_cloud->header.frame_id = "map";
  point_cloud->height = point_cloud->width = 1;

  std::vector<Eigen::Isometry3d> optimized_poses{Eigen::Isometry3d::Identity()};
  for (unsigned int i = 0; i < 14 * IMAGE_SAMPLE_NUM; i++) {
    // Step 2: load images
    auto image = load_next_image_TUM(bp, RGB_TOPIC, true);
    auto depth_image = load_next_image_TUM(bp, DEPTH_TOPIC, false);
    if (i % IMAGE_SAMPLE_NUM != 0)
      continue;
    // Step 3: feature matching, and find depth
    auto orb_res = detect_orb_features(image);
    if (!last_orb_result.is_null()) {
      auto good_matches =
          find_matches_and_draw_them(last_orb_result, orb_res, false);
      // Step 4: find 3D "world frame coordinates". For simplicity, the world
      // frame here is the first camera frame
      PoseEstimate3D estimate_3d = pose_estimate_3d2d_opencv(
          last_orb_result, orb_res, good_matches, cam_info.K, depth_image);
      std::cout << "Before optimization: R " << estimate_3d.R << std::endl;
      std::cout << "Before optimization: t " << estimate_3d.t << std::endl;

      std::vector<Eigen::Vector3d> optimized_points;
      Eigen::Isometry3d pose;
      bundle_adjustment_two_frames(estimate_3d, cam_info, pose,
                                   optimized_points);
      pose = optimized_poses.back() * pose;
      optimized_poses.push_back(pose);
      add_point_cloud(pose, image, depth_image, cam_info, point_cloud);
    }
    last_orb_result = std::move(orb_res);
  }
  visualize_slam_results(optimized_poses, poses_pub, points_pub, point_cloud);
  ros::spin();

  return 0;
}
