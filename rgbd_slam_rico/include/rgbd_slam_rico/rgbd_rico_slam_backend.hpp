#pragma once
#include "rgbd_slam_rico/debug_utils.hpp"
#include "rgbd_slam_rico/orb_feature_detection.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/cv_utils.hpp"
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <random>

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
// 6 is the dimension of poses, 3 is the dimension of landmarks
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>
    SlamLinearSolver;

/**
 * Intro to g2o optimization
 * SparseOptimizer is the graph, which contains multiple edges and vertices. It
 * also needs an optimization algorithm.
 * - Optimization Algorithm: Gauss-Newton, LM, and Powell's dogleg. Optimization
 * Algorithm needs 2 sub solvers: SparseBlockMatrix, and a linear solver.
 * - SparseBlockMatrix is needed for computing Jacobian and Hessina using Schur.
 *
 */

namespace RgbdSlamRico {
g2o::SparseOptimizer global_optimizer;

enum class EdgeAdditionMode { NEARBY, RANDOM };

/**
 * @brief : adding a vertex with frame ID only (estimate doens't matter)
 *
 * @param frame : frame info
 * @return g2o::VertexSE3* : vertex pointer
 */
g2o::VertexSE3 *_get_vertex_ptr(const KeyFrameData &frame) {
  // TODO Can we use VertexSE3Expmap (lie algebra)? VertexSE3 uses tranformation
  // matrix.
  g2o::VertexSE3 *v = new g2o::VertexSE3();
  v->setId(frame.index);
  v->setEstimate(Eigen::Isometry3d::Identity());
  return v;
}

void initialize_global_optimizer(const bool &verbose,
                                 const KeyFrameData &first_frame) {
  // Linear Solver --> Block Solver --> Sparse Optimizer
  auto linear_solver = g2o::make_unique<SlamLinearSolver>();
  // WHY?? TODO
  linear_solver->setBlockOrdering(false);

  auto block_solver =
      g2o::make_unique<SlamBlockSolver>(std::move(linear_solver));
  // global optimizer only takes in a raw pointer
  auto algorithm =
      new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  global_optimizer.setAlgorithm(algorithm);
  global_optimizer.setVerbose(verbose);
  auto v = _get_vertex_ptr(first_frame);
  v->setFixed(true); // Fix the first vertex
  global_optimizer.addVertex(v);
}

// We assume frame 1 has been added to the graph already
void add_single_frame_to_graph(const KeyFrameData &frame1,
                               const KeyFrameData &frame2,
                               const Eigen::Isometry3d &frame1_to_frame2,
                               const bool &add_new_vertex,
                               const std::string &robust_kernel_name) {
  if (add_new_vertex) {
    auto v2 = _get_vertex_ptr(frame2);
    global_optimizer.addVertex(v2);
  }

  g2o::RobustKernel *robust_kernel =
      g2o::RobustKernelFactory::instance()->construct(robust_kernel_name);
  g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
  // Connecting two vertices
  edge->vertices()[0] = global_optimizer.vertex(frame1.index);
  edge->vertices()[1] = global_optimizer.vertex(frame2.index);
  edge->setRobustKernel(robust_kernel);
  if (!edge->vertices()[0] || !edge->vertices()[1]) {
    std::cerr << "=========================================================="
              << std::endl;
    std::cerr << "One of the vertices is not found in the optimizer! First: "
              << (!!edge->vertices()[0]) << " | " << (!!edge->vertices()[1])
              << std::endl;
    std::cerr << "frame1 index: " << frame1.index
              << ", frame2 index: " << frame2.index << std::endl;
    print_vertices(global_optimizer);
  }
  // Information matrix is to estimate how good this edge estimate is.
  // It's the inverse of covariance matrix. It's 6d because a transform is 6d.
  // We assume standard deviation is 0.1, so the covariance is 100.
  Eigen::Matrix<double, 6, 6> information_matrix =
      Eigen::Matrix<double, 6, 6>::Identity();
  information_matrix *= 100;
  edge->setInformation(information_matrix);
  // TODO: "Measurement" means "what you observe of vertex 0 from vertex 2"
  // our frame1_to_frame2 is the observation of frame2 in frame2. So we need to
  // invert it
  edge->setMeasurement(frame1_to_frame2);
  global_optimizer.addEdge(edge);
}

// Important: current frame should NOT be within keyframes.
void add_edges_to_previous_vertices(
    const KeyFrameData &current_keyframe,
    const std::vector<KeyFrameData> &keyframes, const HandyCameraInfo &cam_info,
    const SLAMParams &slam_params, const EdgeAdditionMode &edge_addition_mode) {
  std::vector<unsigned int> frame_indices;
  int start_nearby = std::max(static_cast<int>(keyframes.size()) -
                                  slam_params.nearby_vertex_check_num,
                              0);
  if (edge_addition_mode == EdgeAdditionMode::NEARBY) {
    // keyframes.size()-1 because we already checked the last one
    // vector will be [start, end)
    int end = keyframes.size() - 1;
    for (int i = start_nearby; i <= end; ++i) {
      frame_indices.push_back(i);
    }
  } else if (edge_addition_mode == EdgeAdditionMode::RANDOM) {
    // Max M random numbers [0, start_nearby), TODO
    std::vector<unsigned int> pool;
    for (unsigned int i = 0; i < start_nearby; ++i) {
      pool.push_back(i);
    }

    if (start_nearby <= slam_params.random_vertex_check_num) {
      frame_indices = std::move(pool);
    } else {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::shuffle(pool.begin(), pool.end(), gen);
      frame_indices.insert(frame_indices.end(), pool.begin(),
                           pool.begin() + slam_params.random_vertex_check_num);
    }
  }
  for (const auto &i : frame_indices) {
    auto frame1_to_frame2 =
        front_end(cam_info, slam_params, keyframes.at(i), current_keyframe);
    if (frame1_to_frame2.has_value()) {
      add_single_frame_to_graph(keyframes.at(i), current_keyframe,
                                frame1_to_frame2.value(), false,
                                slam_params.robust_kernel_name);
    }
  }
}

/**
 * @brief Run the global optimizer, and update keyframes with the updated global
 * pose
 *
 * @param keyframes
 */
void backend_optimize(std::vector<KeyFrameData> &keyframes) {
  print_vertices(global_optimizer);
  print_edges(global_optimizer);

  global_optimizer.initializeOptimization();
  global_optimizer.optimize(100);

  for (unsigned int i = 0; i < keyframes.size(); ++i) {
    auto &k = keyframes.at(i);
    g2o::VertexSE3 *v =
        dynamic_cast<g2o::VertexSE3 *>(global_optimizer.vertex(k.index));
    // Does this give the global pose?? TODO
    k.pose = v->estimate();
  }
}
} // namespace RgbdSlamRico
