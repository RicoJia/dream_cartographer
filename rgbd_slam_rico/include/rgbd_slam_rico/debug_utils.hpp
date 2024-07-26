#pragma once
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/io_utils.hpp"
#include "simple_robotics_cpp_utils/thread_pool.hpp"
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
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
namespace RgbdSlamRico {

/////////////////////////////////////////////////////////////////////////////////////////////////
// Point Cloud Utils
/////////////////////////////////////////////////////////////////////////////////////////////////
ThreadPool thread_pool(std::thread::hardware_concurrency() /
                       2); // using half of hardware cores

PointCloud::Ptr initialize_point_cloud() {
  PointCloud::Ptr point_cloud(new PointCloud);
  point_cloud->header.frame_id = "camera";
  point_cloud->height = point_cloud->width = 1;
  // still say is not dense so there might be invalid points
  point_cloud->is_dense = false;
  return point_cloud;
}

// This function might look dumb, but we just don't want the user to worry about
// the nitty gritty of pcl.
void clear_pointcloud(PointCloud::Ptr point_cloud) { point_cloud->clear(); }

/**
 * @brief "Generally thread safe" function to perform the proper pose transforms
 on the current point cloud. Technically, all arguments should be immutables
 across all threads to ensure absolute thread safety
 *
 * @param world_to_cam : transform world -> cam.
 * @param image : RGB image for the current frame
 * @param depth_image : depth image
 * @param cam_info : camera info with intrinsics
 * @param slam_params : slam prarameters
 * @return PointCloud::Ptr : point cloud for the current frame
 */
PointCloud::Ptr
process_current_pointcloud(const Eigen::Isometry3d &world_to_cam,
                           const cv::Mat &image, const cv::Mat &depth_image,
                           const HandyCameraInfo &cam_info,
                           const SLAMParams &slam_params) {
  auto cam_to_world = world_to_cam.inverse();
  //   std::cout << "point cloud addition, pose: " << world_to_cam.matrix()
  //             << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  for (float v = 0; v < depth_image.rows; ++v) {
    for (float u = 0; u < depth_image.cols; ++u) {
      // step 1: get depth
      double depth = depth_image.at<float>(v, u);
      if (std::isnan(depth) || depth < slam_params.min_depth ||
          depth > slam_params.max_depth)
        continue;

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
      current_cloud->points.emplace_back(std::move(pcl_point));
    }
  }

  if (slam_params.downsample_point_cloud) {
    // Create the VoxelGrid filter and set its parameters
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(current_cloud);
    sor.setLeafSize(slam_params.voxel_size, slam_params.voxel_size,
                    slam_params.voxel_size);
    sor.filter(*current_cloud);
  }
  // PointCloud::Ptr is a boost shared pointer and shouldn't cause memory leaks
  return current_cloud;
}

void add_point_cloud(const Eigen::Isometry3d &world_to_cam,
                     const cv::Mat &image, const cv::Mat &depth_image,
                     const HandyCameraInfo &cam_info,
                     PointCloud::Ptr point_cloud,
                     const SLAMParams &slam_params) {

  auto current_cloud = process_current_pointcloud(
      world_to_cam, image, depth_image, cam_info, slam_params);
  *point_cloud += *current_cloud;
  point_cloud->width = point_cloud->points.size();
}

void add_point_cloud_multithreaded(const std::vector<KeyFrameData> &keyframes,
                                   const HandyCameraInfo &cam_info,
                                   PointCloud::Ptr point_cloud,
                                   const SLAMParams &slam_params) {

  std::vector<std::future<PointCloud::Ptr>> futures;
  for (unsigned int i = 0; i < keyframes.size(); ++i) {
    auto &k = keyframes.at(i);
    futures.push_back(thread_pool.enqueue([&] {
      return process_current_pointcloud(k.pose, k.image, k.depth_image,
                                        cam_info, slam_params);
    }));
  }
  for (auto &current_pointcloud_fut : futures) {
    auto current_pointcloud = current_pointcloud_fut.get();
    *point_cloud += *current_pointcloud;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// Visualization Functions
/////////////////////////////////////////////////////////////////////////////////////////////////

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

/**
 * @brief Function to print edges
 *
 * @param optimizer - g2o optimizer
 */
void print_vertices(const g2o::SparseOptimizer &optimizer) {
  std::cout << "Vertices in the optimizer:" << std::endl;
  for (const auto &vertex_pair : optimizer.vertices()) {
    const g2o::OptimizableGraph::Vertex *vertex =
        static_cast<const g2o::OptimizableGraph::Vertex *>(vertex_pair.second);
    std::cout << "Vertex ID: " << vertex->id() << std::endl;
    // Add more details as needed
  }
}

/**
 * @brief Iterate through edges and print them
 *
 * @param optimizer - g2o optimizer
 */
void print_edges(const g2o::SparseOptimizer &optimizer) {
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

class ImagePub {
public:
  ImagePub(ros::NodeHandle nh, const std::string &topic) : it_(nh) {
    pub_ = it_.advertise(topic, 1);
  }

  void publish(const cv::Mat &image) {
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub_.publish(msg);
  }

private:
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;
};
}; // namespace RgbdSlamRico