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
      //  Is it bgr or rgb? TODO
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

void debug_print_3D_points(const PoseEstimate3D &estimate, const HandyCameraInfo &cam_info,
    Eigen::Isometry3d &pose){
    // Transform object frame points (frame1) to frame 2
    std::vector<Eigen::Vector3d> frame1_points, frame2_points;
    frame1_points.resize(estimate.object_frame_points.size());
    frame2_points.resize(estimate.current_camera_pixels.size());
    for (const auto& point : estimate.object_frame_points){
        Eigen::Vector3d p(point.x, point.y, point.z);
        auto transformed_point = pose * p;
        frame1_points.emplace_back(transformed_point);
    }
    // Do these points share the same order?
    // estimate.current_camera_pixels

}

int main(int argc, char *argv[]) {
    // Step 1: set up ros node, and parameters
    ros::init(argc, argv, "rgbd_rico_slam_mini");
    ros::NodeHandle nh;
    int image_skip_batch_num;
    int image_num;
    bool test_with_optimization;
    bool do_ba_two_frames;
    bool do_ba_backend;
    std::string pnp_method;
    nh.getParam("image_skip_batch_num", image_skip_batch_num);
    nh.getParam("image_num", image_num);
    nh.getParam("test_with_optimization", test_with_optimization);
    nh.getParam("pnp_method", pnp_method);
    nh.getParam("do_ba_two_frames", do_ba_two_frames);
    nh.getParam("do_ba_backend", do_ba_backend);
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
    bp.add_topic<sensor_msgs::Image>(RGB_TOPIC, false);
    constexpr auto DEPTH_TOPIC = "/camera/depth/image";
    bp.add_topic<sensor_msgs::Image>(DEPTH_TOPIC, false);
    constexpr auto CAMERA_INFO_TOPIC = "/camera/rgb/camera_info";
    HandyCameraInfo cam_info = load_camera_info(bp, CAMERA_INFO_TOPIC);
    ORBFeatureDetectionResult last_orb_result;

    // Step 3: initialize datastructure for optimization 
    PointCloud::Ptr point_cloud(new PointCloud);
    point_cloud->header.frame_id = "camera";
    point_cloud->height = point_cloud->width = 1;
    std::vector<Eigen::Isometry3d> optimized_poses{Eigen::Isometry3d::Identity()};

    for (unsigned int i = 0; i < image_num * image_skip_batch_num; i++) {
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
                // Step 7: find 3D "world frame coordinates" through pnp. For simplicity, the world
                // frame here is the first camera frame
                PoseEstimate3D estimate_3d =
                    pose_estimate_3d2d_opencv(last_orb_result, orb_res, good_matches,
                                                cam_info.K, depth_image, pnp_method_enum);
                pose = SimpleRoboticsCppUtils::cv_R_t_to_eigen_isometry3d(
                    estimate_3d.R, estimate_3d.t);
                // Step 8: further optimization using local BA
                if (do_ba_two_frames) {
                    std::vector<Eigen::Vector3d> optimized_points;
                    bundle_adjustment_two_frames(estimate_3d, cam_info, pose,
                                                optimized_points);
                }
                pose = optimized_poses.back() * pose;
            }
            // Step 9: prepare output
            // std::cout << "T: " << std::endl << pose.matrix() << std::endl;
            debug_print_3D_points(estimate, cam_info, pose);
            optimized_poses.push_back(pose);
            add_point_cloud(pose, image, depth_image, cam_info, point_cloud);
        }
        last_orb_result = std::move(orb_res);
    }
    // Step 10: global BA optimization (backend)
    if (do_ba_backend) {}
    visualize_slam_results(optimized_poses, poses_pub, points_pub, point_cloud);
    ros::spin();

    return 0;
}
