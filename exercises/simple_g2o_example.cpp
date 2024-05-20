// g++ -std=c++17 simple_g2o_example.cpp -o simple_g2o_example
// -I/usr/include/eigen3 -I/opt/ros/noetic/include -L/opt/ros/noetic/lib
// -lg2o_core -lg2o_stuff -lg2o_solver_dense -lg2o_types_slam2d
// -lg2o_types_slam3d -lg2o_solver_eigen -lg2o_types_sba g2o - General Graph
// Optimization 2011 R. Kuemmerle, G. Grisetti, W. Burgard
/**
 * Problem set up: Graph Optimization is optimization. Graph is just a way to
 represent it.
    - One node is a set of parameters to be optimized. (like a,b,c).
    If you have multiple poses, each pose (x,y,theta), will be an individual
 node
    - One edge is a residual. That is one (yi, xi). You can think of this as a
 single observation.
 * So the general procedure is:
    1. Add a vertex (node)
    2. Add residuals (edges)
    3. Set up solver.
 * Other facts about g2o: it only has numerical differentiation
 */

#include "exercises_common.hpp"
#include <Eigen/Core>
#include <Eigen/Dense> // This includes most of the core Eigen components
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <ostream>

using namespace std;
using namespace g2o;

// 3 is min dimension
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  // This is memory alignment
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Vertex Reset
  virtual void setToOriginImpl() override { _estimate << 0.0, 0.0, 0.0; }

  // Update Vertex - In the curve fitting case, we just simply need to add the
  // update vector because it's in the vector space. However, in SE(3), we need
  // to update it properly
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(
        update); // Ensure _estimate and update are correctly managed
  }

  // file reading and writing
  // Just include virtual and override keywords
  virtual bool read(istream &in) { return true; }
  virtual bool write(ostream &out) const { return true; }

  // Members: estimate() for edge to look up for error computation
};

class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // TODO: is it calling base class ctor here?
  CurveFittingEdge(double x) : BaseUnaryEdge(), x_(x) {}

  // calculating jacobian
  // Has member _vertices, _measurement, _error, _jacobianOplusXi
  virtual void computeError() override {
    const CurveFittingVertex *vertex =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = vertex->estimate();
    _error(0, 0) =
        _measurement - std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
  }

  // calculate jacobian
  virtual void linearizeOplus() override {
    const CurveFittingVertex *vertex =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = vertex->estimate();
    double y_i = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
    _jacobianOplusXi[0] = -x_ * x_ * y_i;
    _jacobianOplusXi[1] = -x_ * y_i;
    _jacobianOplusXi[2] = -y_i;
  }
  virtual bool read(istream &in) { return true; }
  virtual bool write(ostream &out) const { return true; }

private:
  double x_;
};

int main(int argc, char **argv) {
  // error is (3,1)
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;

  // Gradient Descent
  auto new_solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(new_solver);
  // optimizer.setVerbose(true);

  // Adding vertices
  auto v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0.3, 0.4, 0.6));
  v->setId(0);
  optimizer.addVertex(v);

  auto simulated_xy = get_simulated_xy();
  for (int i = 0; i < SIMULATED_DATA_SIZE; ++i) {
    auto [x, y] = simulated_xy[i];
    auto e = new CurveFittingEdge(x);
    e->setId(i);
    // TODO, what does 0 here mean?
    e->setVertex(0, v);
    e->setMeasurement(y);
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 /
                      (std_dev * std_dev));
    optimizer.addEdge(e);
  }
  optimizer.initializeOptimization();
  // what is 10?
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  optimizer.optimize(50);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  auto abc_estimate = v->estimate();
  // 0.0039s
  std::cout << "abc estimate: " << abc_estimate.transpose() << std::endl;
  auto t1_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "optimization time: " << t1_diff.count() << std::endl;
  return 0;
}