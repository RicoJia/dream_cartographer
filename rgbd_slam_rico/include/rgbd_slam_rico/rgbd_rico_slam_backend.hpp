#pragma once
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace RgbdSlamRico {

/************************************** Constants
 * **************************************/

/************************************** Data Structures
 * **************************************/
/************************************** Functions
 * **************************************/

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
// adding one camera pose as a vertex
inline void
_add_camera_pose(const PoseEstimate3D &estimate, const unsigned int &vertex_id,
                 g2o::SparseOptimizer &optimizer,
                 std::vector<g2o::VertexSE3Expmap *> &camera_pose_vertices) {
  g2o::VertexSE3Expmap *v_se3 = new g2o::VertexSE3Expmap();
  camera_pose_vertices.push_back(v_se3);
  v_se3->setId(vertex_id);
  // This could be more efficient
  Eigen::Isometry3d transform =
      SimpleRoboticsCppUtils::cv_R_t_to_eigen_isometry3d(estimate.R,
                                                         estimate.t);
  g2o::SE3Quat se3quat(transform.rotation(), transform.translation());
  v_se3->setEstimate(se3quat);
  optimizer.addVertex(v_se3);
}

// adding 3D points as vertices, 2D points as projection points
inline void
_add_3D_2D_points(const PoseEstimate3D &estimate, unsigned int &vertex_id,
                  g2o::SparseOptimizer &optimizer,
                  std::vector<g2o::VertexSBAPointXYZ *> &point3d_vertices) {
  // vertex_id is the index of the camera pose.
  unsigned int camera_pose_vertex_id = vertex_id;
  for (unsigned int i = 0; i < estimate.object_frame_points.size(); i++) {
    const cv::Point3f &p = estimate.object_frame_points.at(i);
    const cv::Point2f &p_2d = estimate.current_camera_pixels.at(i);

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

// inline void bundle_adjustment(
//     const std::vector<PoseEstimate3D>& estimates,
//     const HandyCameraInfo& cam_info,
//     std::vector<Eigen::Isometry3d>& optimized_poses,
//     std::vector<Eigen::Vector3d>& optimized_points
// ){
// }

inline void bundle_adjustment_two_frames(
    const PoseEstimate3D &estimate, const HandyCameraInfo &cam_info,
    Eigen::Isometry3d &pose, std::vector<Eigen::Vector3d> &optimized_points) {
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
  _add_camera_pose(estimate, vertex_id, optimizer, camera_pose_vertices);
  _add_3D_2D_points(estimate, vertex_id, optimizer, point3d_vertices);

  // TODO What does optimizer.optimize(10);mean?
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // TODO: should we push all points here?
  // TODO: should we add all edges between the 3D point to multiple camera poses
  pose = Eigen::Isometry3d(camera_pose_vertices.at(0)->estimate());
  for (const auto &point : point3d_vertices) {
    optimized_points.push_back(point->estimate());
  }
}

} // namespace RgbdSlamRico
