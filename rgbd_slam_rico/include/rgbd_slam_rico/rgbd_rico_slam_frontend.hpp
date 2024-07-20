#pragma once
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace RgbdSlamRico {

/************************************** Data Structures
 * **************************************/
struct SLAMParams {
  double max_depth = 20.0;
  double min_depth = 0.3;
  bool do_ba_two_frames = true;
  bool verbose = false;
  bool do_ba_backend = true;
  bool pause_after_optimization = false;
};

struct FrontEndData {
  cv::Mat image;
  cv::Mat depth_image;
  std::vector<cv::Point3f> object_frame_points;
  std::vector<cv::Point2f> current_camera_pixels;
};

/************************************** Functions - 2D 2D
 * **************************************/

inline int read_pnp_method(const std::string &method) {
  if (method == "epnp")
    return cv::SOLVEPNP_EPNP;
  else if (method == "p3p")
    return cv::SOLVEPNP_P3P;
  else if (method == "dls")
    return cv::SOLVEPNP_DLS;
  else
    return cv::SOLVEPNP_EPNP;
}

/**
 * @brief filter out matches with invalid depths IN PLACE, i.e., depth outside
 * of [min_threshold, max_threshold]
 *
 * @param matches : point matches across two RGB images
 * @param depth1 : depth image of the first RGB image
 * @param depth2 : depth image of the second RGB image
 */
inline void filter_point_matches_with_valid_depths(
    std::vector<cv::DMatch> &matches, const cv::Mat &depth_img1,
    const cv::Mat &depth_img2, const ORBFeatureDetectionResult &res1,
    const ORBFeatureDetectionResult &res2, const SLAMParams &slam_params) {
  // NEED TO TEST??
  auto remove_invalid_matches = [&](const cv::DMatch &match) {
    auto p1 = res1.keypoints.at(match.queryIdx).pt;
    double depth1 = depth_img1.at<float>(int(p1.y), int(p1.x));
    if (std::isnan(depth1) || depth1 < slam_params.min_depth ||
        depth1 > slam_params.max_depth)
      return true;

    auto p2 = res2.keypoints.at(match.trainIdx).pt;
    double depth2 = depth_img2.at<float>(int(p2.y), int(p2.x));
    if (std::isnan(depth2) || depth2 < slam_params.min_depth ||
        depth2 > slam_params.max_depth)
      return true;

    return false;
  };
  auto new_end =
      std::remove_if(matches.begin(), matches.end(), remove_invalid_matches);
  matches.erase(new_end, matches.end());
}

/************************************** Functions - 3D 2D
 * **************************************/

/**
 * @brief Get data ready for Motion Estimation using 3D-2D correspondence
 *
 * @param object_frame_points : output 3D matched points in the object (world)
 * frame
 * @param current_camera_points : output 2D matched pixel in the current camera
 * frame
 * @param res1 : feature detection result of the previous image
 * @param res2 : feature detection result of the current camera image
 * @param feature_matches : OpenCV feature matching
 * @param K : camera intrinsics
 * @param depth1 : depth image associated with the previous image.
 */
inline void
get_object_and_2d_points(std::vector<cv::Point3f> &object_frame_points,
                         std::vector<cv::Point2f> &current_camera_pixels,
                         const ORBFeatureDetectionResult &res1,
                         const ORBFeatureDetectionResult &res2,
                         const std::vector<cv::DMatch> &feature_matches,
                         const cv::Mat &K, const cv::Mat &depth1,
                         const SLAMParams &slam_params) {
  for (const cv::DMatch &match : feature_matches) {
    auto previous_pt = res1.keypoints.at(match.queryIdx).pt;
    double depth = depth1.at<float>(int(previous_pt.y), int(previous_pt.x));
    if (std::isnan(depth) || depth < slam_params.min_depth ||
        depth > slam_params.max_depth)
      continue; // bad depth
    // to canonical form, then to depth
    auto p_canonical = SimpleRoboticsCppUtils::pixel2cam(previous_pt, K);
    object_frame_points.emplace_back(p_canonical.x * depth,
                                     p_canonical.y * depth, depth);
    current_camera_pixels.emplace_back(res2.keypoints.at(match.trainIdx).pt);
  }
}

inline void _add_intrinsics(const HandyCameraInfo &cam_info,
                            g2o::SparseOptimizer &optimizer) {
  auto K_eigen =
      SimpleRoboticsCppUtils::cv_R_to_eigen_matrixXd<Eigen::Matrix3d>(
          cam_info.K);
  // TODO: what does below mean?
  g2o::CameraParameters *params = new g2o::CameraParameters(
      K_eigen(0, 0), Eigen::Vector2d(K_eigen(0, 2), K_eigen(1, 2)), 0);
  params->setId(0);
  optimizer.addParameter(params);
}
// // adding one camera pose as a vertex
inline void
_add_camera_pose(const Eigen::Isometry3d &frame1_to_frame2,
                 const unsigned int &vertex_id, g2o::SparseOptimizer &optimizer,
                 std::vector<g2o::VertexSE3Expmap *> &camera_pose_vertices) {
  g2o::VertexSE3Expmap *v_se3 = new g2o::VertexSE3Expmap();
  camera_pose_vertices.push_back(v_se3);
  v_se3->setId(vertex_id);
  // This could be more efficient
  g2o::SE3Quat se3quat(frame1_to_frame2.rotation(),
                       frame1_to_frame2.translation());
  v_se3->setEstimate(se3quat);
  optimizer.addVertex(v_se3);
}

// adding 3D points as vertices, 2D points as projection points
inline void
_add_3D_2D_points(const FrontEndData &front_end_data, unsigned int &vertex_id,
                  g2o::SparseOptimizer &optimizer,
                  std::vector<g2o::VertexSBAPointXYZ *> &point3d_vertices) {
  // vertex_id is the index of the camera pose.
  unsigned int camera_pose_vertex_id = vertex_id;
  for (unsigned int i = 0; i < front_end_data.object_frame_points.size(); i++) {
    const cv::Point3f &p = front_end_data.object_frame_points.at(i);
    const cv::Point2f &p_2d = front_end_data.current_camera_pixels.at(i);

    ++vertex_id;
    // adding 3D points as vertices
    g2o::VertexSBAPointXYZ *v_p = new g2o::VertexSBAPointXYZ();
    point3d_vertices.push_back(v_p);
    v_p->setId(vertex_id);
    v_p->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
    v_p->setMarginalized(true);
    optimizer.addVertex(v_p);

    /*
    0th vertex (from) is the 3D point, 1th vertex (to) is the camera pose
    */
    g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
    edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                           optimizer.vertex(vertex_id)));
    edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                           optimizer.vertex(camera_pose_vertex_id)));
    edge->setMeasurement(Eigen::Vector2d{p_2d.x, p_2d.y});
    edge->setInformation(Eigen::Matrix2d::Identity());
    edge->setParameterId(0, 0);
    optimizer.addEdge(edge);
  }
  // for the next guy
  ++vertex_id;
}

inline void bundle_adjustment_two_frames(
    const FrontEndData &front_end_data, const HandyCameraInfo &cam_info,
    Eigen::Isometry3d &frame1_to_frame2,
    std::vector<Eigen::Vector3d> &optimized_points, const bool &verbose) {
  // TODO: other optimizers other than the sparse? What does it do?
  // TODO: what is BlockSolver?
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<g2o::BlockSolverX>(
          g2o::make_unique<
              g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>())));

  _add_intrinsics(cam_info, optimizer);

  std::vector<g2o::VertexSE3Expmap *> camera_pose_vertices;
  std::vector<g2o::VertexSBAPointXYZ *> point3d_vertices;

  unsigned int vertex_id = 0;
  // Step 2: adding camera poses as vertices.
  _add_camera_pose(frame1_to_frame2, vertex_id, optimizer,
                   camera_pose_vertices);
  _add_3D_2D_points(front_end_data, vertex_id, optimizer, point3d_vertices);

  // TODO What does optimizer.optimize(10);mean?
  optimizer.setVerbose(verbose);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // Not pushing all points here because this is enough points!
  frame1_to_frame2 = Eigen::Isometry3d(camera_pose_vertices.at(0)->estimate());
  for (const auto &point : point3d_vertices) {
    optimized_points.push_back(point->estimate());
  }
}

Eigen::Isometry3d front_end(const ORBFeatureDetectionResult &res1,
                            const ORBFeatureDetectionResult &res2,
                            const std::vector<cv::DMatch> &feature_matches,
                            FrontEndData &front_end_data,
                            const int &pnp_method_enum,
                            const SLAMParams &slam_params,
                            const HandyCameraInfo &cam_info) {
  // Step 1 - PnP
  const cv::Mat &depth1 = front_end_data.depth_image;
  const cv::Mat &K = cam_info.K;
  get_object_and_2d_points(front_end_data.object_frame_points,
                           front_end_data.current_camera_pixels, res1, res2,
                           feature_matches, K, depth1, slam_params);

  /**
   cv::SOLVEPNP_DLS: "A Direct Least-Squares (DLS) Method for PnP"
  cv::SOLVEPNP_P3P : "Complete Solution Classification for the
  Perspective-Three-Point Problem". It needs 4 points exactly cv::SOLVEPNP_EPNP
  : "Complete Solution Classification for the Perspective-Three-Point Problem".
  It needs 4 points exactly cv::Mat() is distCoeffs
  */
  // TODO: to add our custom patches.
  cv::Mat r, t;
  cv::solvePnP(front_end_data.object_frame_points,
               front_end_data.current_camera_pixels, K, cv::Mat(), r, t, false,
               pnp_method_enum);
  cv::Mat R;
  cv::Rodrigues(r, R);
  Eigen::Isometry3d frame1_to_frame2 =
      SimpleRoboticsCppUtils::cv_R_t_to_eigen_isometry3d(R, t);

  // Step 2 - Local Bundle Adjustment

  // find 3D "world frame coordinates" through pnp. For simplicity, the world
  // frame here is the first camera frame
  if (slam_params.do_ba_two_frames) {
    std::vector<Eigen::Vector3d> optimized_points;
    bundle_adjustment_two_frames(front_end_data, cam_info, frame1_to_frame2,
                                 optimized_points, slam_params.verbose);
  }
  return frame1_to_frame2;
}

} // namespace RgbdSlamRico
