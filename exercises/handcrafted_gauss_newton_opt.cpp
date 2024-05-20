// g++ handcrafted_gauss_newton_opt.cpp -lceres -I /usr/include/eigen3 -lglog
// --std=c++17
#include "exercises_common.hpp"
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/types.h>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <glog/logging.h>
#include <tuple>
#include <vector>

// Alternative: you can explore manually using Jacobian
struct CostFunctor {
  // Pass your data point here.
  CostFunctor(const double &x, const double &y) : x_(x), y_(y) {}
  const double x_;
  const double y_;

  // 1. ceres::Jet can get directional direvatives
  // this is the cost function, which is y - exp(ax^2 + bx + c). params here are
  // [a,b,c] why are we using T? - because it could be ceres::Jet, or double
  // 2. Residual is the total error term
  // 3. Autodiff will do its optimization during compile time. It's faster and
  // more accurate than numerical diff because numerical diff is not templated
  template <typename T> bool operator()(const T *params, T *residual) const {
    // These types are implicitly converted into ceres::Jet. so std::exp won't
    // work here
    residual[0] =
        y_ - ceres::exp(params[0] * x_ * x_ + params[1] * x_ + params[2]);
    // why are we returning true? Because there could be corner cases where
    // residual is not easily computed
    return true;
  }
};

void ceres_optimization(
    const std::vector<std::tuple<double, double>> &simulated_xy,
    double *params_estimates) {

  ceres::Problem problem;

  for (unsigned int i = 0; i < simulated_xy.size(); i++) {
    auto [x, y] = simulated_xy[i];
    // all params to be optimized are called a "param block"
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 3>(
                                 new CostFunctor(x, y)),
                             nullptr, params_estimates);
  }

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << "Summary:\n" << summary.BriefReport() << "\n";
}

void handcrafted_optimization(
    const std::vector<std::tuple<double, double>> &simulated_xy,
    double *params_estimates) {
  // inv(sum(J*J^T))(-sum(J)e)
  constexpr unsigned int N = 50;
  double inv_sigma_squared = 1 / std_dev * 1 / std_dev;
  double last_cost = 0;
  for (unsigned int i = 0; i < N; ++i) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    double cost = 0;
    for (const auto &tup : simulated_xy) {
      auto [x, y] = tup;
      double f0 = std::exp(params_estimates[0] * x * x +
                           params_estimates[1] * x + params_estimates[2]);
      Eigen::Vector3d J{-x * x * f0, -x * f0, -f0};

      H += inv_sigma_squared * J * J.transpose();
      double error = y - f0;
      b += -J * error * inv_sigma_squared;
      cost += error * error;
    }
    // iterate over all datapoints
    // get J
    // get sum of J
    // Get step, update the params

    if (cost > last_cost && i > 0) {
      // TODO
      std::cout << "Break! Cost: " << cost << std::endl;
      break;
    }
    last_cost = cost;
    Eigen::Vector3d dx = H.ldlt().solve(b);
    if (std::isnan(dx[0])) {
      // TODO Why only checking 0
      std::cout << "Result is nan!" << std::endl;
      break;
    }
    params_estimates[0] += dx[0];
    params_estimates[1] += dx[1];
    params_estimates[2] += dx[2];
  }
}

//
int main(int argc, char **argv) {
  auto simulated_xy = get_simulated_xy();
  for (const auto &tup : simulated_xy) {
    auto [x, y] = tup;
    std::cout << "x, y: " << x << "," << y << std::endl;
  }

  // TODO what does this do?
  google::InitGoogleLogging(argv[0]);
  // The variable to solve for with its initial value.
  double a = 0.3;
  double b = 0.4;
  double c = 6;
  double params_estimates[3] = {a, b, c};

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres_optimization(simulated_xy, params_estimates);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  params_estimates[0] = a;
  params_estimates[1] = b;
  params_estimates[2] = c;
  handcrafted_optimization(simulated_xy, params_estimates);
  std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
  for (const auto &param : params_estimates)
    std::cout << param << " ";
  std::cout << std::endl;
  auto t1_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  auto t2_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
  // Handcrafted optimization takes 0.002, ceres takes 0.003, and did not
  // converge...
  std::cout << "ceres_optimization time: " << t1_diff.count() << std::endl;
  std::cout << "handcrafted time: " << t2_diff.count() << std::endl;

  return 0;
}