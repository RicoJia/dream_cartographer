// Adopted from https://blog.csdn.net/leonardohaig/article/details/120756834
#include <cmath>
#include <cstring>
#include <iostream>

#include "rgbd_slam_rico/p3p.hpp"

void p3p::init_inverse_parameters() {
  inv_fx = 1. / fx;
  inv_fy = 1. / fy;
  cx_fx = cx / fx;
  cy_fy = cy / fy;
}

p3p::p3p(cv::Mat cameraMatrix) {
  if (cameraMatrix.depth() == CV_32F)
    init_camera_parameters<float>(cameraMatrix);
  else
    init_camera_parameters<double>(cameraMatrix);
  init_inverse_parameters();
}

p3p::p3p(double _fx, double _fy, double _cx, double _cy) {
  fx = _fx;
  fy = _fy;
  cx = _cx;
  cy = _cy;
  init_inverse_parameters();
}

bool p3p::solve(cv::Mat &R, cv::Mat &tvec, const cv::Mat &opoints,
                const cv::Mat &ipoints) {
  double rotation_matrix[3][3] = {}, translation[3] = {};
  std::vector<double> points;
  if (opoints.depth() == ipoints.depth()) {
    if (opoints.depth() == CV_32F)
      extract_points<cv::Point3f, cv::Point2f>(opoints, ipoints, points);
    else
      extract_points<cv::Point3d, cv::Point2d>(opoints, ipoints, points);
  } else if (opoints.depth() == CV_32F)
    extract_points<cv::Point3f, cv::Point2d>(opoints, ipoints, points);
  else
    extract_points<cv::Point3d, cv::Point2f>(opoints, ipoints, points);

  bool result = solve(rotation_matrix, translation, points[0], points[1],
                      points[2], points[3], points[4], points[5], points[6],
                      points[7], points[8], points[9], points[10], points[11],
                      points[12], points[13], points[14], points[15],
                      points[16], points[17], points[18], points[19]);
  cv::Mat(3, 1, CV_64F, translation).copyTo(tvec);
  cv::Mat(3, 3, CV_64F, rotation_matrix).copyTo(R);
  return result;
}

int p3p::solve(std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &tvecs,
               const cv::Mat &opoints, const cv::Mat &ipoints) {
  double rotation_matrix[4][3][3] = {}, translation[4][3] = {};
  std::vector<double> points;
  if (opoints.depth() == ipoints.depth()) {
    if (opoints.depth() == CV_32F)
      extract_points<cv::Point3f, cv::Point2f>(opoints, ipoints, points);
    else
      extract_points<cv::Point3d, cv::Point2d>(opoints, ipoints, points);
  } else if (opoints.depth() == CV_32F)
    extract_points<cv::Point3f, cv::Point2d>(opoints, ipoints, points);
  else
    extract_points<cv::Point3d, cv::Point2f>(opoints, ipoints, points);

  const bool p4p = std::max(opoints.checkVector(3, CV_32F),
                            opoints.checkVector(3, CV_64F)) == 4;
  int solutions = solve(rotation_matrix, translation, points[0], points[1],
                        points[2], points[3], points[4], points[5], points[6],
                        points[7], points[8], points[9], points[10], points[11],
                        points[12], points[13], points[14], points[15],
                        points[16], points[17], points[18], points[19], p4p);

  for (int i = 0; i < solutions; i++) {
    cv::Mat R, tvec;
    cv::Mat(3, 1, CV_64F, translation[i]).copyTo(tvec);
    cv::Mat(3, 3, CV_64F, rotation_matrix[i]).copyTo(R);

    Rs.push_back(R);
    tvecs.push_back(tvec);
  }

  return solutions;
}

bool p3p::solve(double R[3][3], double t[3], double mu0, double mv0, double X0,
                double Y0, double Z0, double mu1, double mv1, double X1,
                double Y1, double Z1, double mu2, double mv2, double X2,
                double Y2, double Z2, double mu3, double mv3, double X3,
                double Y3, double Z3) {
  double Rs[4][3][3] = {}, ts[4][3] = {};

  const bool p4p = true;
  int n = solve(Rs, ts, mu0, mv0, X0, Y0, Z0, mu1, mv1, X1, Y1, Z1, mu2, mv2,
                X2, Y2, Z2, mu3, mv3, X3, Y3, Z3, p4p);

  if (n == 0)
    return false;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      R[i][j] = Rs[0][i][j];
    t[i] = ts[0][i];
  }

  return true;
}

int p3p::solve(double R[4][3][3], double t[4][3], double mu0, double mv0,
               double X0, double Y0, double Z0, double mu1, double mv1,
               double X1, double Y1, double Z1, double mu2, double mv2,
               double X2, double Y2, double Z2, double mu3, double mv3,
               double X3, double Y3, double Z3, bool p4p) {
  if (0) {
    //验证:利用公式(28)的矩阵形式计算余弦
    cv::Mat x1 = (cv::Mat_<double>(3, 1) << mu0, mv0, 1);
    cv::Mat x2 = (cv::Mat_<double>(3, 1) << mu1, mv1, 1);

    //内参矩阵
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    cv::Mat X = K.inv() * x1; //公式29(b)或者说是(26b)
    std::cout << "lihengXXXX:" << X << std::endl;

    cv::Mat H = (K.t()).inv() * K.inv();
    cv::Mat numerator = x1.t() * H * x2; //公式(28)的分子部分
    cv::Mat denominator_2 =
        (x1.t() * H * x1) * (x2.t() * H * x2); //公式(28)的 分母^2 部分

    auto C12 =
        numerator.at<double>(0, 0) / std::sqrt(denominator_2.at<double>(0, 0));
    std::cout << C12 << std::endl;
  }

  double mk0, mk1, mk2;
  double norm;

  //求解归一化图像平面上的坐标--公式(26);
  // 然后单位化--公式(27)
  mu0 = inv_fx * mu0 - cx_fx;
  mv0 = inv_fy * mv0 - cy_fy; //求解归一化图像平面上坐标:Z_C=1的平面
  norm = sqrt(mu0 * mu0 + mv0 * mv0 + 1); //求模长
  mk0 = 1. / norm;
  mu0 *= mk0;
  mv0 *= mk0; //转换为单位向量(单位坐标)

  mu1 = inv_fx * mu1 - cx_fx;
  mv1 = inv_fy * mv1 - cy_fy;
  norm = sqrt(mu1 * mu1 + mv1 * mv1 + 1);
  mk1 = 1. / norm;
  mu1 *= mk1;
  mv1 *= mk1;

  mu2 = inv_fx * mu2 - cx_fx;
  mv2 = inv_fy * mv2 - cy_fy;
  norm = sqrt(mu2 * mu2 + mv2 * mv2 + 1);
  mk2 = 1. / norm;
  mu2 *= mk2;
  mv2 *= mk2;

  mu3 = inv_fx * mu3 - cx_fx;
  mv3 = inv_fy * mv3 - cy_fy;

  double distances[3];
  distances[0] = sqrt((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2) +
                      (Z1 - Z2) * (Z1 - Z2)); // d12
  distances[1] = sqrt((X0 - X2) * (X0 - X2) + (Y0 - Y2) * (Y0 - Y2) +
                      (Z0 - Z2) * (Z0 - Z2)); // d02
  distances[2] = sqrt((X0 - X1) * (X0 - X1) + (Y0 - Y1) * (Y0 - Y1) +
                      (Z0 - Z1) * (Z0 - Z1)); // d01

  // Calculate angles
  //公式(24) 向量内积,计算角度
  double cosines[3];
  cosines[0] = mu1 * mu2 + mv1 * mv2 + mk1 * mk2; // C12
  cosines[1] = mu0 * mu2 + mv0 * mv2 + mk0 * mk2; // C02
  cosines[2] = mu0 * mu1 + mv0 * mv1 + mk0 * mk1; // C01

  double lengths[4][3] = {};
  int n = my_solve_for_lengths(lengths, distances,
                               cosines); //自己推导实现的长度计算
  // n = solve_for_lengths(lengths, distances, cosines);//OpenCV源码中长度计算

  //利用公式(30) 计算点在相机坐标系下的坐标
  int nb_solutions = 0;
  double reproj_errors[4];
  for (int i = 0; i < n; i++) {
    double M_orig[3][3];

    M_orig[0][0] = lengths[i][0] * mu0;
    M_orig[0][1] = lengths[i][0] * mv0;
    M_orig[0][2] = lengths[i][0] * mk0;

    M_orig[1][0] = lengths[i][1] * mu1;
    M_orig[1][1] = lengths[i][1] * mv1;
    M_orig[1][2] = lengths[i][1] * mk1;

    M_orig[2][0] = lengths[i][2] * mu2;
    M_orig[2][1] = lengths[i][2] * mv2;
    M_orig[2][2] = lengths[i][2] * mk2;

    // ICP 求解外参矩阵
    if (!align(M_orig, X0, Y0, Z0, X1, Y1, Z1, X2, Y2, Z2, R[nb_solutions],
               t[nb_solutions]))
      continue;

    //利用第4个点辅助选择
    if (p4p) {
      double X3p = R[nb_solutions][0][0] * X3 + R[nb_solutions][0][1] * Y3 +
                   R[nb_solutions][0][2] * Z3 + t[nb_solutions][0];
      double Y3p = R[nb_solutions][1][0] * X3 + R[nb_solutions][1][1] * Y3 +
                   R[nb_solutions][1][2] * Z3 + t[nb_solutions][1];
      double Z3p = R[nb_solutions][2][0] * X3 + R[nb_solutions][2][1] * Y3 +
                   R[nb_solutions][2][2] * Z3 + t[nb_solutions][2];
      double mu3p = X3p / Z3p;
      double mv3p = Y3p / Z3p;
      reproj_errors[nb_solutions] =
          (mu3p - mu3) * (mu3p - mu3) + (mv3p - mv3) * (mv3p - mv3);
    }

    nb_solutions++;
  }

  if (p4p) {
    // sort the solutions
    for (int i = 1; i < nb_solutions; i++) {
      for (int j = i; j > 0 && reproj_errors[j - 1] > reproj_errors[j]; j--) {
        std::swap(reproj_errors[j], reproj_errors[j - 1]);
        std::swap(R[j], R[j - 1]);
        std::swap(t[j], t[j - 1]);
      }
    }
  }

  return nb_solutions;
}

/// Given 3D distances between three points and cosines of 3 angles at the apex,
/// calculates the lengths of the line segments connecting projection center (P)
/// and the three 3D points (A, B, C). Returned distances are for |PA|, |PB|,
/// |PC| respectively. Only the solution to the main branch. Reference : X.S.
/// Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for
/// the Perspective-Three-Point Problem" IEEE Trans. on PAMI, vol. 25, No. 8,
/// August 2003 \param lengths3D Lengths of line segments up to four solutions.
/// \param dist3D Distance between 3D points in pairs |BC|, |AC|, |AB|.
/// \param cosines Cosine of the angles /_BPC, /_APC, /_APB.
/// \returns Number of solutions.
/// WARNING: NOT ALL THE DEGENERATE CASES ARE IMPLEMENTED

int p3p::solve_for_lengths(double lengths[4][3], double distances[3],
                           double cosines[3]) {
  double p = cosines[0] * 2;
  double q = cosines[1] * 2;
  double r = cosines[2] * 2;

  double inv_d22 = 1. / (distances[2] * distances[2]);
  double a = inv_d22 * (distances[0] * distances[0]);
  double b = inv_d22 * (distances[1] * distances[1]);

  double a2 = a * a, b2 = b * b, p2 = p * p, q2 = q * q, r2 = r * r;
  double pr = p * r, pqr = q * pr;

  // Check reality condition (the four points should not be coplanar)
  if (p2 + q2 + r2 - pqr - 1 == 0)
    return 0;

  double ab = a * b, a_2 = 2 * a;

  double A = -2 * b + b2 + a2 + 1 + ab * (2 - r2) - a_2;

  // Check reality condition
  if (A == 0)
    return 0;

  double a_4 = 4 * a;

  double B = q * (-2 * (ab + a2 + 1 - b) + r2 * ab + a_4) + pr * (b - b2 + ab);
  double C = q2 + b2 * (r2 + p2 - 2) - b * (p2 + pqr) - ab * (r2 + pqr) +
             (a2 - a_2) * (2 + q2) + 2;
  double D = pr * (ab - b2 + b) + q * ((p2 - 2) * b + 2 * (ab - a2) + a_4 - 2);
  double E = 1 + 2 * (b - a - ab) + b2 - b * p2 + a2;

  double temp = (p2 * (a - 1 + b) + r2 * (a - 1 - b) + pqr - a * pqr);
  double b0 = b * temp * temp;
  // Check reality condition
  if (b0 == 0)
    return 0;

  double real_roots[4];
  int n = solve_deg4(A, B, C, D, E, real_roots[0], real_roots[1], real_roots[2],
                     real_roots[3]);

  if (n == 0)
    return 0;

  int nb_solutions = 0;
  double r3 = r2 * r, pr2 = p * r2, r3q = r3 * q;
  double inv_b0 = 1. / b0;

  // For each solution of x
  for (int i = 0; i < n; i++) {
    double x = real_roots[i];

    // Check reality condition
    if (x <= 0)
      continue;

    double x2 = x * x;

    double b1 =
        ((1 - a - b) * x2 + (q * a - q) * x + 1 - a + b) *
        (((r3 * (a2 + ab * (2 - r2) - a_2 + b2 - 2 * b + 1)) * x +

          (r3q * (2 * (b - a2) + a_4 + ab * (r2 - 2) - 2) +
           pr2 * (1 + a2 + 2 * (ab - a - b) + r2 * (b - b2) + b2))) *
             x2 +

         (r3 * (q2 * (1 - 2 * a + a2) + r2 * (b2 - ab) - a_4 + 2 * (a2 - b2) +
                2) +
          r * p2 * (b2 + 2 * (ab - b - a) + 1 + a2) +
          pr2 * q * (a_4 + 2 * (b - ab - a2) - 2 - r2 * b)) *
             x +

         2 * r3q * (a_2 - b - a2 + ab - 1) +
         pr2 * (q2 - a_4 + 2 * (a2 - b2) + r2 * b + q2 * (a2 - a_2) + 2) +
         p2 * (p * (2 * (ab - a - b) + a2 + b2 + 1) +
               2 * q * r * (b + a_2 - a2 - ab - 1)));

    // Check reality condition
    if (b1 <= 0)
      continue;

    double y = inv_b0 * b1;
    double v = x2 + y * y - x * y * r;

    if (v <= 0)
      continue;

    double Z = distances[2] / sqrt(v);
    double X = x * Z;
    double Y = y * Z;

    lengths[nb_solutions][0] = X;
    lengths[nb_solutions][1] = Y;
    lengths[nb_solutions][2] = Z;

    nb_solutions++;
  }

  return nb_solutions;
}

int p3p::my_solve_for_lengths(double lengths[4][3], double distances[3],
                              double cosines[3]) {
  //传进来的数据的下标顺序为12 02 01
  //对应推导公式中的23 13 12

  // 0 1 2
  const auto d23 = distances[0];
  const auto d13 = distances[1];
  const auto d12 = distances[2];

  const auto C23 = cosines[0];
  const auto C13 = cosines[1];
  const auto C12 = cosines[2];

  const auto C23_2 = C23 * C23;
  const auto C13_2 = C13 * C13;
  const auto C12_2 = C12 * C12;
  const auto C123 = C23 * C13 * C12;

  //判断点是否共面
  // Check reality condition (the four points should not be coplanar)
  if (C23_2 + C13_2 + C12_2 - 2 * C123 - 1 ==
      0) // OpenCV p3p源码中该部分可能有误
    return 0;

  // TODO:判断距离是否为0 提前返回
  //求解系数--公式(13)
  const auto K1 = std::pow(d23 / d13, 2);
  const auto K2 = std::pow(d23 / d12, 2);

  const auto G4 = std::pow(K1 * K2 - K1 - K2, 2) - 4 * K1 * K2 * C23_2;
  // Check reality condition
  if (G4 == 0)
    return 0;

  const auto G3 =
      4 * (K1 * K2 - K1 - K2) * K2 * (1 - K1) * C12 +
      4 * K1 * C23 * ((K1 * K2 - K1 + K2) * C13 + 2 * K2 * C12 * C23);

  const auto G2 = std::pow(2 * K2 * (1 - K1) * C12, 2) +
                  2 * (K1 * K2 - K1 - K2) * (K1 * K2 + K1 - K2) +
                  4 * K1 *
                      ((K1 - K2) * C23_2 + K1 * (1 - K2) * C13_2 -
                       2 * (1 + K1) * K2 * C123);

  const auto G1 =
      4 * (K1 * K2 + K1 - K2) * K2 * (1 - K1) * C12 +
      4 * K1 * ((K1 * K2 - K1 + K2) * C13 * C23 + 2 * K1 * K2 * C12 * C13_2);
  const auto G0 = std::pow(K1 * K2 + K1 - K2, 2) - 4 * K1 * K1 * K2 * C13_2;

  //求解公式(12)中x的解
  double real_roots[4] = {0};
  int n = solve_deg4(G4, G3, G2, G1, G0, real_roots[0], real_roots[1],
                     real_roots[2], real_roots[3]);

  if (n == 0)
    return 0;

  int nb_solutions = 0;

  // For each solution of x cal y,and then d1,d2,d3
  for (int i = 0; i < n; i++) {
    const double x = real_roots[i];

    // Check reality condition
    if (x <= 0)
      continue;

    //利用公式(14)(15)求解y
    const double H1 = 2 * K1 * (C13 - C23 * x);
    if (H1 == 0)
      continue;

    const double H0 =
        x * x - K1 - (K1 - 1) * ((K2 - 1) * x * x - 2 * C12 * K2 * x + K2);
    if (H0 == 0)
      continue;

    const double y = -1 * H0 / H1;

    //求解d1--公式(4a)
    const double v = 1 + x * x - 2 * x * C12;
    if (v <= 0)
      continue;
    const double d1 = d12 / std::sqrt(v);

    // d2 d3
    const double d2 = x * d1;
    const double d3 = y * d1;

    lengths[nb_solutions][0] = d1;
    lengths[nb_solutions][1] = d2;
    lengths[nb_solutions][2] = d3;

    nb_solutions++;
  }

  return nb_solutions;
}

bool p3p::align(double M_end[3][3], double X0, double Y0, double Z0, double X1,
                double Y1, double Z1, double X2, double Y2, double Z2,
                double R[3][3], double T[3]) {
  // Centroids:
  double C_start[3] = {}, C_end[3] = {};
  for (int i = 0; i < 3; i++)
    C_end[i] = (M_end[0][i] + M_end[1][i] + M_end[2][i]) / 3;
  C_start[0] = (X0 + X1 + X2) / 3;
  C_start[1] = (Y0 + Y1 + Y2) / 3;
  C_start[2] = (Z0 + Z1 + Z2) / 3;

  // Covariance matrix s:
  double s[3 * 3] = {};
  for (int j = 0; j < 3; j++) {
    s[0 * 3 + j] =
        (X0 * M_end[0][j] + X1 * M_end[1][j] + X2 * M_end[2][j]) / 3 -
        C_end[j] * C_start[0];
    s[1 * 3 + j] =
        (Y0 * M_end[0][j] + Y1 * M_end[1][j] + Y2 * M_end[2][j]) / 3 -
        C_end[j] * C_start[1];
    s[2 * 3 + j] =
        (Z0 * M_end[0][j] + Z1 * M_end[1][j] + Z2 * M_end[2][j]) / 3 -
        C_end[j] * C_start[2];
  }

  double Qs[16] = {}, evs[4] = {}, U[16] = {};

  Qs[0 * 4 + 0] = s[0 * 3 + 0] + s[1 * 3 + 1] + s[2 * 3 + 2];
  Qs[1 * 4 + 1] = s[0 * 3 + 0] - s[1 * 3 + 1] - s[2 * 3 + 2];
  Qs[2 * 4 + 2] = s[1 * 3 + 1] - s[2 * 3 + 2] - s[0 * 3 + 0];
  Qs[3 * 4 + 3] = s[2 * 3 + 2] - s[0 * 3 + 0] - s[1 * 3 + 1];

  Qs[1 * 4 + 0] = Qs[0 * 4 + 1] = s[1 * 3 + 2] - s[2 * 3 + 1];
  Qs[2 * 4 + 0] = Qs[0 * 4 + 2] = s[2 * 3 + 0] - s[0 * 3 + 2];
  Qs[3 * 4 + 0] = Qs[0 * 4 + 3] = s[0 * 3 + 1] - s[1 * 3 + 0];
  Qs[2 * 4 + 1] = Qs[1 * 4 + 2] = s[1 * 3 + 0] + s[0 * 3 + 1];
  Qs[3 * 4 + 1] = Qs[1 * 4 + 3] = s[2 * 3 + 0] + s[0 * 3 + 2];
  Qs[3 * 4 + 2] = Qs[2 * 4 + 3] = s[2 * 3 + 1] + s[1 * 3 + 2];

  jacobi_4x4(Qs, evs, U);

  // Looking for the largest eigen value:
  int i_ev = 0;
  double ev_max = evs[i_ev];
  for (int i = 1; i < 4; i++)
    if (evs[i] > ev_max)
      ev_max = evs[i_ev = i];

  // Quaternion:
  double q[4];
  for (int i = 0; i < 4; i++)
    q[i] = U[i * 4 + i_ev];

  double q02 = q[0] * q[0], q12 = q[1] * q[1], q22 = q[2] * q[2],
         q32 = q[3] * q[3];
  double q0_1 = q[0] * q[1], q0_2 = q[0] * q[2], q0_3 = q[0] * q[3];
  double q1_2 = q[1] * q[2], q1_3 = q[1] * q[3];
  double q2_3 = q[2] * q[3];

  R[0][0] = q02 + q12 - q22 - q32;
  R[0][1] = 2. * (q1_2 - q0_3);
  R[0][2] = 2. * (q1_3 + q0_2);

  R[1][0] = 2. * (q1_2 + q0_3);
  R[1][1] = q02 + q22 - q12 - q32;
  R[1][2] = 2. * (q2_3 - q0_1);

  R[2][0] = 2. * (q1_3 - q0_2);
  R[2][1] = 2. * (q2_3 + q0_1);
  R[2][2] = q02 + q32 - q12 - q22;

  for (int i = 0; i < 3; i++)
    T[i] = C_end[i] -
           (R[i][0] * C_start[0] + R[i][1] * C_start[1] + R[i][2] * C_start[2]);

  return true;
}

bool p3p::jacobi_4x4(double *A, double *D, double *U) {
  double B[4] = {}, Z[4] = {};
  double Id[16] = {1., 0., 0., 0., 0., 1., 0., 0.,
                   0., 0., 1., 0., 0., 0., 0., 1.};

  memcpy(U, Id, 16 * sizeof(double));

  B[0] = A[0];
  B[1] = A[5];
  B[2] = A[10];
  B[3] = A[15];
  memcpy(D, B, 4 * sizeof(double));

  for (int iter = 0; iter < 50; iter++) {
    double sum = fabs(A[1]) + fabs(A[2]) + fabs(A[3]) + fabs(A[6]) +
                 fabs(A[7]) + fabs(A[11]);

    if (sum == 0.0)
      return true;

    double tresh = (iter < 3) ? 0.2 * sum / 16. : 0.0;
    for (int i = 0; i < 3; i++) {
      double *pAij = A + 5 * i + 1;
      for (int j = i + 1; j < 4; j++) {
        double Aij = *pAij;
        double eps_machine = 100.0 * fabs(Aij);

        if (iter > 3 && fabs(D[i]) + eps_machine == fabs(D[i]) &&
            fabs(D[j]) + eps_machine == fabs(D[j]))
          *pAij = 0.0;
        else if (fabs(Aij) > tresh) {
          double hh = D[j] - D[i], t;
          if (fabs(hh) + eps_machine == fabs(hh))
            t = Aij / hh;
          else {
            double theta = 0.5 * hh / Aij;
            t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
            if (theta < 0.0)
              t = -t;
          }

          hh = t * Aij;
          Z[i] -= hh;
          Z[j] += hh;
          D[i] -= hh;
          D[j] += hh;
          *pAij = 0.0;

          double c = 1.0 / sqrt(1 + t * t);
          double s = t * c;
          double tau = s / (1.0 + c);
          for (int k = 0; k <= i - 1; k++) {
            double g = A[k * 4 + i], h = A[k * 4 + j];
            A[k * 4 + i] = g - s * (h + g * tau);
            A[k * 4 + j] = h + s * (g - h * tau);
          }
          for (int k = i + 1; k <= j - 1; k++) {
            double g = A[i * 4 + k], h = A[k * 4 + j];
            A[i * 4 + k] = g - s * (h + g * tau);
            A[k * 4 + j] = h + s * (g - h * tau);
          }
          for (int k = j + 1; k < 4; k++) {
            double g = A[i * 4 + k], h = A[j * 4 + k];
            A[i * 4 + k] = g - s * (h + g * tau);
            A[j * 4 + k] = h + s * (g - h * tau);
          }
          for (int k = 0; k < 4; k++) {
            double g = U[k * 4 + i], h = U[k * 4 + j];
            U[k * 4 + i] = g - s * (h + g * tau);
            U[k * 4 + j] = h + s * (g - h * tau);
          }
        }
        pAij++;
      }
    }

    for (int i = 0; i < 4; i++)
      B[i] += Z[i];
    memcpy(D, B, 4 * sizeof(double));
    memset(Z, 0, 4 * sizeof(double));
  }

  return false;
}

//==================================================================//
// OpenCV源码路径 opencv/modules/calib3d/src/polynom_solver.cpp
int solve_deg2(double a, double b, double c, double &x1, double &x2) {
  double delta = b * b - 4 * a * c;

  if (delta < 0)
    return 0;

  double inv_2a = 0.5 / a;

  if (delta == 0) {
    x1 = -b * inv_2a;
    x2 = x1;
    return 1;
  }

  double sqrt_delta = sqrt(delta);
  x1 = (-b + sqrt_delta) * inv_2a;
  x2 = (-b - sqrt_delta) * inv_2a;
  return 2;
}

/// Reference : Eric W. Weisstein. "Cubic Equation." From MathWorld--A Wolfram
/// Web Resource. http://mathworld.wolfram.com/CubicEquation.html \return Number
/// of real roots found.
int solve_deg3(double a, double b, double c, double d, double &x0, double &x1,
               double &x2) {
  if (a == 0) {
    // Solve second order system
    if (b == 0) {
      // Solve first order system
      if (c == 0)
        return 0;

      x0 = -d / c;
      return 1;
    }

    x2 = 0;
    return solve_deg2(b, c, d, x0, x1);
  }

  // Calculate the normalized form x^3 + a2 * x^2 + a1 * x + a0 = 0
  double inv_a = 1. / a;
  double b_a = inv_a * b, b_a2 = b_a * b_a;
  double c_a = inv_a * c;
  double d_a = inv_a * d;

  // Solve the cubic equation
  double Q = (3 * c_a - b_a2) / 9;
  double R = (9 * b_a * c_a - 27 * d_a - 2 * b_a * b_a2) / 54;
  double Q3 = Q * Q * Q;
  double D = Q3 + R * R;
  double b_a_3 = (1. / 3.) * b_a;

  if (Q == 0) {
    if (R == 0) {
      x0 = x1 = x2 = -b_a_3;
      return 3;
    } else {
      x0 = pow(2 * R, 1 / 3.0) - b_a_3;
      return 1;
    }
  }

  if (D <= 0) {
    // Three real roots
    double theta = acos(R / sqrt(-Q3));
    double sqrt_Q = sqrt(-Q);
    x0 = 2 * sqrt_Q * cos(theta / 3.0) - b_a_3;
    x1 = 2 * sqrt_Q * cos((theta + 2 * CV_PI) / 3.0) - b_a_3;
    x2 = 2 * sqrt_Q * cos((theta + 4 * CV_PI) / 3.0) - b_a_3;

    return 3;
  }

  // D > 0, only one real root
  double AD =
      pow(fabs(R) + sqrt(D), 1.0 / 3.0) * (R > 0 ? 1 : (R < 0 ? -1 : 0));
  double BD = (AD == 0) ? 0 : -Q / AD;

  // Calculate the only real root
  x0 = AD + BD - b_a_3;

  return 1;
}

/// Reference : Eric W. Weisstein. "Quartic Equation." From MathWorld--A Wolfram
/// Web Resource. http://mathworld.wolfram.com/QuarticEquation.html \return
/// Number of real roots found.
int solve_deg4(double a, double b, double c, double d, double e, double &x0,
               double &x1, double &x2, double &x3) {
  if (a == 0) {
    x3 = 0;
    return solve_deg3(b, c, d, e, x0, x1, x2);
  }

  // Normalize coefficients
  double inv_a = 1. / a;
  b *= inv_a;
  c *= inv_a;
  d *= inv_a;
  e *= inv_a;
  double b2 = b * b, bc = b * c, b3 = b2 * b;

  // Solve resultant cubic
  double r0, r1, r2;
  int n =
      solve_deg3(1, -c, d * b - 4 * e, 4 * c * e - d * d - b2 * e, r0, r1, r2);
  if (n == 0)
    return 0;

  // Calculate R^2
  double R2 = 0.25 * b2 - c + r0, R;
  if (R2 < 0)
    return 0;

  R = sqrt(R2);
  double inv_R = 1. / R;

  int nb_real_roots = 0;

  // Calculate D^2 and E^2
  double D2, E2;
  if (R < 10E-12) {
    double temp = r0 * r0 - 4 * e;
    if (temp < 0)
      D2 = E2 = -1;
    else {
      double sqrt_temp = sqrt(temp);
      D2 = 0.75 * b2 - 2 * c + 2 * sqrt_temp;
      E2 = D2 - 4 * sqrt_temp;
    }
  } else {
    double u = 0.75 * b2 - 2 * c - R2, v = 0.25 * inv_R * (4 * bc - 8 * d - b3);
    D2 = u + v;
    E2 = u - v;
  }

  double b_4 = 0.25 * b, R_2 = 0.5 * R;
  if (D2 >= 0) {
    double D = sqrt(D2);
    nb_real_roots = 2;
    double D_2 = 0.5 * D;
    x0 = R_2 + D_2 - b_4;
    x1 = x0 - D;
  }

  // Calculate E^2
  if (E2 >= 0) {
    double E = sqrt(E2);
    double E_2 = 0.5 * E;
    if (nb_real_roots == 0) {
      x0 = -R_2 + E_2 - b_4;
      x1 = x0 - E;
      nb_real_roots = 2;
    } else {
      x2 = -R_2 + E_2 - b_4;
      x3 = x2 - E;
      nb_real_roots = 4;
    }
  }

  return nb_real_roots;
}

int SolveP3P(cv::InputArray _opoints, cv::InputArray _ipoints,
             cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
             cv::OutputArrayOfArrays _rvecs, cv::OutputArrayOfArrays _tvecs,
             int flags) {
  // opoint is world frame, ipoints is pixel?
  cv::Mat opoints = _opoints.getMat(), ipoints = _ipoints.getMat();
  int npoints =
      std::max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));
  CV_Assert(npoints == std::max(ipoints.checkVector(2, CV_32F),
                                ipoints.checkVector(2, CV_64F)));
  CV_Assert(npoints == 3 || npoints == 4);
  CV_Assert(flags == cv::SOLVEPNP_P3P); //仅扩展P3P 未扩展AP3P!!!!!

  if (opoints.cols == 3)
    opoints = opoints.reshape(3);
  if (ipoints.cols == 2)
    ipoints = ipoints.reshape(2);

  cv::Mat cameraMatrix0 = _cameraMatrix.getMat();
  cv::Mat distCoeffs0 = _distCoeffs.getMat();
  cv::Mat cameraMatrix = cv::Mat_<double>(cameraMatrix0);
  cv::Mat distCoeffs = cv::Mat_<double>(distCoeffs0);

  //畸变校正.求无畸变时点的位置
  //针对鱼眼镜头,应采用的函数为 cv::fisheye::undistortPoints()
  //或者,在调用该函数之前进行畸变校正,传参时将畸变设为0;
  //此处调用时已将cameraMatrix传参传入!!!
  cv::Mat undistortedPoints;
  cv::undistortPoints(ipoints, undistortedPoints, cameraMatrix, distCoeffs,
                      cv::Mat(), cameraMatrix);

  std::vector<cv::Mat> Rs, ts, rvecs;

  int solutions = 0;
  if (flags == cv::SOLVEPNP_P3P) {
    p3p P3Psolver(cameraMatrix);
    solutions = P3Psolver.solve(Rs, ts, opoints, undistortedPoints);
  } else //未扩展
  {
  }

  if (solutions == 0) {
    return 0;
  }

  cv::Mat objPts, imgPts;
  opoints.convertTo(objPts, CV_64F);
  ipoints.convertTo(imgPts, CV_64F);
  if (imgPts.cols > 1) {
    imgPts = imgPts.reshape(1);
    imgPts = imgPts.t();
  } else
    imgPts = imgPts.reshape(1, 2 * imgPts.rows);

  std::vector<double> reproj_errors(solutions);
  for (size_t i = 0; i < reproj_errors.size(); i++) {
    cv::Mat rvec;
    cv::Rodrigues(Rs[i], rvec);
    rvecs.push_back(rvec);

    cv::Mat projPts;
    cv::projectPoints(objPts, rvec, ts[i], _cameraMatrix, _distCoeffs,
                      projPts); //将空间点利用外参矩阵投影到图像上

    projPts = projPts.reshape(1, 2 * projPts.rows);
    cv::Mat err = imgPts - projPts; //计算重投影点和输入点之间的误差:重投影误差

    err = err.t() * err;
    reproj_errors[i] = err.at<double>(0, 0);
  }

  // sort the solutions
  for (int i = 1; i < solutions; i++) {
    for (int j = i; j > 0 && reproj_errors[j - 1] > reproj_errors[j]; j--) {
      std::swap(reproj_errors[j], reproj_errors[j - 1]);
      std::swap(rvecs[j], rvecs[j - 1]);
      std::swap(ts[j], ts[j - 1]);
    }
  }

  int depthRot = _rvecs.fixedType() ? _rvecs.depth() : CV_64F;
  int depthTrans = _tvecs.fixedType() ? _tvecs.depth() : CV_64F;
  _rvecs.create(
      solutions, 1,
      CV_MAKETYPE(depthRot, _rvecs.fixedType() &&
                                    _rvecs.kind() == cv::_InputArray::STD_VECTOR
                                ? 3
                                : 1));
  _tvecs.create(solutions, 1,
                CV_MAKETYPE(depthTrans,
                            _tvecs.fixedType() &&
                                    _tvecs.kind() == cv::_InputArray::STD_VECTOR
                                ? 3
                                : 1));

  for (int i = 0; i < solutions; i++) {
    cv::Mat rvec0, tvec0;
    if (depthRot == CV_64F)
      rvec0 = rvecs[i];
    else
      rvecs[i].convertTo(rvec0, depthRot);

    if (depthTrans == CV_64F)
      tvec0 = ts[i];
    else
      ts[i].convertTo(tvec0, depthTrans);

    if (_rvecs.fixedType() && _rvecs.kind() == cv::_InputArray::STD_VECTOR) {
      cv::Mat rref = _rvecs.getMat_();

      if (_rvecs.depth() == CV_32F)
        rref.at<cv::Vec3f>(0, i) =
            cv::Vec3f(rvec0.at<float>(0, 0), rvec0.at<float>(1, 0),
                      rvec0.at<float>(2, 0));
      else
        rref.at<cv::Vec3d>(0, i) =
            cv::Vec3d(rvec0.at<double>(0, 0), rvec0.at<double>(1, 0),
                      rvec0.at<double>(2, 0));
    } else {
      _rvecs.getMatRef(i) = rvec0;
    }

    if (_tvecs.fixedType() && _tvecs.kind() == cv::_InputArray::STD_VECTOR) {

      cv::Mat tref = _tvecs.getMat_();

      if (_tvecs.depth() == CV_32F)
        tref.at<cv::Vec3f>(0, i) =
            cv::Vec3f(tvec0.at<float>(0, 0), tvec0.at<float>(1, 0),
                      tvec0.at<float>(2, 0));
      else
        tref.at<cv::Vec3d>(0, i) =
            cv::Vec3d(tvec0.at<double>(0, 0), tvec0.at<double>(1, 0),
                      tvec0.at<double>(2, 0));
    } else {
      _tvecs.getMatRef(i) = tvec0;
    }
  }

  return solutions;
}

int main() {
  // 1920*1080
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 983.349f, 0, 959.5f, 0,
                          984.953f, 539.5f, 0, 0, 1);
  cv::Mat distCoeffs =
      (cv::Mat_<double>(5, 1) << -0.0069f, -0.0174f, 0.0045f, 0, 0);
  //    cv::Mat distCoeffs = (cv::Mat_<double >(5,1) << 0,0,0,0,0);

  std::vector<cv::Point3d> objectPoints(3);
  objectPoints[0] = cv::Point3f(-1405, 260, 0);
  objectPoints[1] = cv::Point3f(-415, 354, 0);
  objectPoints[2] = cv::Point3f(-1405, 448, 0);

  std::vector<cv::Point2d> imagePoints(3);
  imagePoints[0] = cv::Point2f(506.95f, 609.08f);
  imagePoints[1] = cv::Point2f(763.5f, 623.3f);
  imagePoints[2] = cv::Point2f(511.12f, 659.56f);

  int nSolver = 0;
  std::vector<cv::Mat> rvecs, tvecs;

  std::cout << "OpenCV's result:" << std::endl;

  //第4个点
  // objectPoints.emplace_back(-910,542,0);
  // imagePoints.emplace_back(634.82,681.63);

  std::vector<cv::Mat>().swap(rvecs);
  std::vector<cv::Mat>().swap(tvecs);
  nSolver = cv::solveP3P(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                         rvecs, tvecs, cv::SOLVEPNP_P3P);
  for (int i = 0; i < nSolver; ++i) {
    printf("rvecs[%d]=\n", i);
    std::cout << rvecs[i] << std::endl;
  }
  for (int i = 0; i < nSolver; ++i) {
    printf("tvecs[%d]=\n", i);
    std::cout << tvecs[i] << std::endl;
  }

  std::cout << "our's result:" << std::endl;
  std::vector<cv::Mat>().swap(rvecs);
  std::vector<cv::Mat>().swap(tvecs);
  nSolver = SolveP3P(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs,
                     tvecs, cv::SOLVEPNP_P3P);
  for (int i = 0; i < nSolver; ++i) {
    printf("rvecs[%d]=\n", i);
    std::cout << rvecs[i] << std::endl;
  }
  for (int i = 0; i < nSolver; ++i) {
    printf("tvecs[%d]=\n", i);
    std::cout << tvecs[i] << std::endl;
  }

  return 0;
}
