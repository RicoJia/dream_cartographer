#pragma once
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/io_utils.hpp"
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Define the IO format to print on one line
Eigen::IOFormat eigen_1_line_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                 ", ", ", ", "", "", " [", "] ");
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace RgbdSlamRico;

void visualize_slam_results(const std::vector<KeyFrameData> &keyframes,
                            ros::Publisher &poses_publisher,
                            ros::Publisher &point_cloud_publisher,
                            PointCloud::Ptr point_cloud) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "camera";

  for (const auto &f : keyframes) {
    const auto &pose = f.pose;
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
      // step 2: get canonical coords, and 3D point
      auto p_canonical = SimpleRoboticsCppUtils::pixel2cam({u, v}, cam_info.K);
      Eigen::Vector3d point{p_canonical.x * depth, p_canonical.y * depth,
                            depth};
      // SolvePnP gives world -> camera world_to_cam.
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

  if (slam_params.downsample_point_cloud) {
    // Create a downsampled point cloud object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    // Create the VoxelGrid filter and set its parameters
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(point_cloud);
    sor.setLeafSize(slam_params.voxel_size, slam_params.voxel_size,
                    slam_params.voxel_size);
    sor.filter(*downsampled_cloud);
    // TODO: not sure if this boost shared pointer will cause memory leaks
    point_cloud->clear();
    *point_cloud = *downsampled_cloud;
  }
}

// Function to print vertices
void printVertices(const g2o::SparseOptimizer &optimizer) {
  std::cout << "Vertices in the optimizer:" << std::endl;
  for (const auto &vertex_pair : optimizer.vertices()) {
    const g2o::OptimizableGraph::Vertex *vertex =
        static_cast<const g2o::OptimizableGraph::Vertex *>(vertex_pair.second);
    std::cout << "Vertex ID: " << vertex->id() << std::endl;
    // Add more details as needed
  }
}

// Function to print edges
void printEdges(const g2o::SparseOptimizer &optimizer) {
  std::cout << "Edges in the optimizer:" << std::endl;
  for (const auto &edge : optimizer.edges()) {
    const g2o::OptimizableGraph::Edge *e =
        static_cast<const g2o::OptimizableGraph::Edge *>(edge);
    std::cout << "Edge ID: " << e->id() << ", connects vertices ";
    for (size_t i = 0; i < e->vertices().size(); ++i) {
      const g2o::OptimizableGraph::Vertex *v =
          static_cast<const g2o::OptimizableGraph::Vertex *>(e->vertices()[i]);
      if (v) {
        std::cout << v->id();
        if (i < e->vertices().size() - 1) {
          std::cout << " -> ";
        }
      }
    }
    std::cout << std::endl;
    // Add more details as needed
  }
}

// /**
//  * @brief converts frame1 2d matches points -> frame 2 2d matches ->
//  canonical -> 3d -> frame 1
//  *
//  * @param estimate
//  * @param cam_info
//  * @param cam1_2_cam2
//  * @param depth2
//  */
// void debug_print_3d_points(const poseestimate3d &estimate, const
// handycamerainfo &cam_info,
//     eigen::isometry3d &cam1_2_cam2, const cv::mat &depth2){

//     std::vector<eigen::vector3d> frame1_points, frame2_points;

//     for (unsigned int i = 0; i < estimate.object_frame_points.size(); ++i){
//         const auto& point1 = estimate.object_frame_points.at(i);
//         eigen::vector3d p1(point1.x, point1.y, point1.z);
//         auto p1_trans = cam1_2_cam2 * p1;

//         const auto& pixel2 = estimate.current_camera_pixels.at(i);
//         auto p2_canonical = simpleroboticscpputils::pixel2cam(pixel2,
//         cam_info.k);

//         double depth = depth2.at<float>(int(pixel2.y), int(pixel2.x));
//         //todo
//         std::cout<<"depth2: "<<depth<<std::endl;
//         std::cout<<"k: "<<cam_info.k<<std::endl;
//         eigen::vector3d p2(p2_canonical.x * depth, p2_canonical.y * depth,
//         depth); std::cout<<"p1: "<<p1_trans.format(eigen_1_line_fmt)<<"p2:
//         "<<p2.format(eigen_1_line_fmt)<<std::endl;
//     }
// }