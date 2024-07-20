#include "rgbd_slam_rico/rgbd_rico_slam_backend.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/io_utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace RgbdSlamRico;
// Define the IO format to print on one line
Eigen::IOFormat eigen_1_line_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                 ", ", ", ", "", "", " [", "] ");

// TODO: move to simple robotics utils
void visualize_slam_results(
    const std::vector<Eigen::Isometry3d> &optimized_poses,
    ros::Publisher &poses_publisher, ros::Publisher &point_cloud_publisher,
    PointCloud::Ptr point_cloud) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "camera";

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
  std::cout << "Visualizing number of points: " << point_cloud->points.size()
            << std::endl;
}

void add_point_cloud(const Eigen::Isometry3d &world_to_cam,
                     const cv::Mat &image, const cv::Mat &depth_image,
                     const HandyCameraInfo &cam_info,
                     PointCloud::Ptr point_cloud,
                     const SLAMParams &slam_params) {
  auto cam_to_world = world_to_cam.inverse();
  std::cout << "point cloud addition, pose: " << world_to_cam.matrix()
            << std::endl;
  for (float v = 0; v < depth_image.rows; ++v) {
    for (float u = 0; u < depth_image.rows; ++u) {
      // step 1: get depth
      double depth = depth_image.at<float>(v, u);
      if (std::isnan(depth) || depth < slam_params.min_depth ||
          depth > slam_params.max_depth)
        continue; // bad depth

      // step 2: get canonical coords, and 3D point
      auto p_canonical = SimpleRoboticsCppUtils::pixel2cam({u, v}, cam_info.K);
      Eigen::Vector3d point{p_canonical.x * depth, p_canonical.y * depth,
                            depth};
      // SolvePnP gives world -> camera world_to_cam. TODO: so we need to
      // inverse this?
      auto world_point = cam_to_world * point;
      // Again, welcome to the BGR world
      cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);
      pcl::PointXYZRGB pcl_point;
      pcl_point.x = world_point[0];
      pcl_point.y = world_point[1];
      pcl_point.z = world_point[2];
      pcl_point.r = bgr[2];
      pcl_point.g = bgr[1];
      pcl_point.b = bgr[0];
      point_cloud->points.push_back(pcl_point);
    }
  }
  point_cloud->width = point_cloud->points.size();
  // still say is not dense so there might be invalid points
  point_cloud->is_dense = false;
}

// /**
//  * @brief Converts frame1 2D matches points -> frame 2 2D matches ->
//  canonical -> 3D -> frame 1
//  *
//  * @param estimate
//  * @param cam_info
//  * @param cam1_2_cam2
//  * @param depth2
//  */
// void debug_print_3D_points(const PoseEstimate3D &estimate, const
// HandyCameraInfo &cam_info,
//     Eigen::Isometry3d &cam1_2_cam2, const cv::Mat &depth2){

//     std::vector<Eigen::Vector3d> frame1_points, frame2_points;

//     for (unsigned int i = 0; i < estimate.object_frame_points.size(); ++i){
//         const auto& point1 = estimate.object_frame_points.at(i);
//         Eigen::Vector3d p1(point1.x, point1.y, point1.z);
//         auto p1_trans = cam1_2_cam2 * p1;

//         const auto& pixel2 = estimate.current_camera_pixels.at(i);
//         auto p2_canonical = SimpleRoboticsCppUtils::pixel2cam(pixel2,
//         cam_info.K);

//         double depth = depth2.at<float>(int(pixel2.y), int(pixel2.x));
//         //TODO
//         std::cout<<"depth2: "<<depth<<std::endl;
//         std::cout<<"K: "<<cam_info.K<<std::endl;
//         Eigen::Vector3d p2(p2_canonical.x * depth, p2_canonical.y * depth,
//         depth); std::cout<<"p1: "<<p1_trans.format(eigen_1_line_fmt)<<"p2:
//         "<<p2.format(eigen_1_line_fmt)<<std::endl;
//     }

// }

int main(int argc, char *argv[]) {
  // Step 1: set up ros node, and parameters
  ros::init(argc, argv, "rgbd_rico_slam_mini");
  ros::NodeHandle nh;
  int image_skip_batch_num;
  int image_num;
  bool test_with_optimization;
  std::string pnp_method;
  SLAMParams slam_params;
  nh.getParam("image_skip_batch_num", image_skip_batch_num);
  nh.getParam("image_num", image_num);
  nh.getParam("test_with_optimization", test_with_optimization);
  nh.getParam("pnp_method", pnp_method);
  nh.getParam("do_ba_two_frames", slam_params.do_ba_two_frames);
  nh.getParam("do_ba_backend", slam_params.do_ba_backend);
  nh.getParam("min_depth", slam_params.min_depth);
  nh.getParam("max_depth", slam_params.max_depth);
  nh.getParam("verbose", slam_params.verbose);
  nh.getParam("pause_after_optimization", slam_params.pause_after_optimization);

  int pnp_method_enum = read_pnp_method(pnp_method);

  // Step 2: bagparser, topics and camera info
  ros::Publisher poses_pub =
      nh.advertise<geometry_msgs::PoseArray>("optimized_poses", 1, true);
  ros::Publisher points_pub =
      nh.advertise<PointCloud>("optimized_points", 1, true);
  constexpr auto PACKAGE_NAME = "rgbd_slam_rico";
  SimpleRoboticsRosUtils::BagParser bp(nh, PACKAGE_NAME,
                                       "/data/rgbd_dataset_freiburg1_xyz.bag");
  constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
  constexpr auto DEPTH_TOPIC = "/camera/depth/image";
  constexpr auto CAMERA_INFO_TOPIC = "/camera/rgb/camera_info";
  HandyCameraInfo cam_info = load_camera_info(bp, CAMERA_INFO_TOPIC);

  // Step 3: initialize data structures for optimization
  PointCloud::Ptr point_cloud(new PointCloud);
  point_cloud->header.frame_id = "camera";
  point_cloud->height = point_cloud->width = 1;
  std::vector<Eigen::Isometry3d> optimized_poses{Eigen::Isometry3d::Identity()};
  ORBFeatureDetectionResult last_orb_result;
  FrontEndData front_end_data;

  for (unsigned int i = 0; i < image_num * image_skip_batch_num; i++) {
    if (!ros::ok())
      break;
    // Step 4: load images
    auto image = load_next_image_TUM(bp, RGB_TOPIC, true);
    auto depth_image = load_next_image_TUM(bp, DEPTH_TOPIC, false);
    if (i % image_skip_batch_num != 0)
      continue;
    // Step 5: feature Detection
    auto orb_res = detect_orb_features(image);
    if (!last_orb_result.is_null()) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      if (test_with_optimization) {
        // Step 6: Match features
        auto good_matches =
            find_matches_and_draw_them(last_orb_result, orb_res, false);
        filter_point_matches_with_valid_depths(
            good_matches, front_end_data.depth_image, depth_image,
            last_orb_result, orb_res, slam_params);
        Eigen::Isometry3d frame1_to_frame2 =
            front_end(last_orb_result, orb_res, good_matches, front_end_data,
                      pnp_method_enum, slam_params, cam_info);
        pose = optimized_poses.back() * frame1_to_frame2;
        // if (verbose) debug_print_3D_points(estimate_3d, cam_info, pose,
        // depth_image);
      }
      // Step 9: prepare output
      optimized_poses.push_back(pose);
      add_point_cloud(pose, image, depth_image, cam_info, point_cloud,
                      slam_params);
      if (slam_params.pause_after_optimization) {
        const std::string msg =
            "Iteration " + std::to_string(i) + ", Please hit enter pause";
        SimpleRoboticsCppUtils::sleep_or_pause_on_keystroke(
            SimpleRoboticsCppUtils::to_ANSI_code(
                SimpleRoboticsCppUtils::ANSIStrings::RED, msg),
            1500);
      }
      // Step 10: global BA optimization (backend)
      visualize_slam_results(optimized_poses, poses_pub, points_pub,
                             point_cloud);
    }
    last_orb_result = std::move(orb_res);
    front_end_data = FrontEndData{std::move(image), std::move(depth_image)};
  }

  ros::spin();

  return 0;
}
