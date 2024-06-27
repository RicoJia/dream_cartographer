#pragma once
#include <memory>
#include <random>

constexpr unsigned int SIMULATED_DATA_SIZE = 35;
constexpr double std_dev = 0.1;
constexpr double A = 0.1;
constexpr double B = 0.2;
constexpr double C = 0.3;

inline double draw_from_pdf_normal(const double &mean, const double &std_dev) {
  // random seed
  static std::random_device rd;
  // Mersenne twister PRNG, seeded with rd
  static std::mt19937 gen(rd());
  std::normal_distribution<double> d(mean, std_dev);
  return d(gen);
}

// y = exp(ax^2 + bx + c) + w
std::vector<std::tuple<double, double>> get_simulated_xy() {
  std::vector<std::tuple<double, double>> ret_vec(SIMULATED_DATA_SIZE);
  double x = -5.0;
  for (auto &tup : ret_vec) {
    x++;
    auto y = std::exp(A * x * x + B * x + C) + draw_from_pdf_normal(0, std_dev);
    tup = std::make_tuple(x, y);
  }
  return ret_vec;
}