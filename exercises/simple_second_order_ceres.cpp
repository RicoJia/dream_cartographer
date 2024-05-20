#include <ceres/ceres.h>
#include <iostream>

// Ceres is a least squares optimizer.
// The "Cost function" that needs to be minimized.
// It models the function (10 - x)^2.
struct CostFunctor {
  template <typename T> bool operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value.
  double initial_x = 5.0;
  double x = initial_x;

  // Build the problem.
  ceres::Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // automatic differentiation to obtain the derivative (jacobian).
  ceres::CostFunction *cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "Summary:\n" << summary.BriefReport() << "\n";
  std::cout << "Initial x: " << initial_x << " -> Optimized x: " << x
            << std::endl;

  return 0;
}