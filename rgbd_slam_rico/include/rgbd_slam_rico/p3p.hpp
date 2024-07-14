// Adopted from https://blog.csdn.net/leonardohaig/article/details/120756834
// Created by liheng on 10/2/21.
// Program:自定义实现P3P算法
// Date:2021-10-2
// Author:liheng
// Version:V1.0
// Note:框架来源于OpenCV源码,但是相机到空间点距离求解部分未采用论文中的消元法
//      修改为自己的推导过程
//====================================================================//

#ifndef OPENCVDEMO_P3P_H
#define OPENCVDEMO_P3P_H

#include <opencv2/opencv.hpp>

class p3p {
public:
  p3p(double fx, double fy, double cx, double cy);
  p3p(cv::Mat cameraMatrix);

  bool solve(cv::Mat &R, cv::Mat &tvec, const cv::Mat &opoints,
             const cv::Mat &ipoints);
  int solve(std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &tvecs,
            const cv::Mat &opoints, const cv::Mat &ipoints);
  int solve(double R[4][3][3], double t[4][3], double mu0, double mv0,
            double X0, double Y0, double Z0, double mu1, double mv1, double X1,
            double Y1, double Z1, double mu2, double mv2, double X2, double Y2,
            double Z2, double mu3, double mv3, double X3, double Y3, double Z3,
            bool p4p);
  bool solve(double R[3][3], double t[3], double mu0, double mv0, double X0,
             double Y0, double Z0, double mu1, double mv1, double X1, double Y1,
             double Z1, double mu2, double mv2, double X2, double Y2, double Z2,
             double mu3, double mv3, double X3, double Y3, double Z3);

private:
  template <typename T>
  void init_camera_parameters(const cv::Mat &cameraMatrix) {
    cx = cameraMatrix.at<T>(0, 2);
    cy = cameraMatrix.at<T>(1, 2);
    fx = cameraMatrix.at<T>(0, 0);
    fy = cameraMatrix.at<T>(1, 1);
  }
  template <typename OpointType, typename IpointType>
  void extract_points(const cv::Mat &opoints, const cv::Mat &ipoints,
                      std::vector<double> &points) {
    points.clear();
    int npoints = std::max(opoints.checkVector(3, CV_32F),
                           opoints.checkVector(3, CV_64F));
    points.resize(5 * 4); // resize vector to fit for p4p case
    for (int i = 0; i < npoints; i++) {
      //          points[i*5] = ipoints.at<IpointType>(i).x*fx + cx;
      //          points[i*5+1] = ipoints.at<IpointType>(i).y*fy + cy;
      //          points[i*5+2] = opoints.at<OpointType>(i).x;
      //          points[i*5+3] = opoints.at<OpointType>(i).y;
      //          points[i*5+4] = opoints.at<OpointType>(i).z;

      points[i * 5] = ipoints.at<IpointType>(i).x;
      points[i * 5 + 1] = ipoints.at<IpointType>(i).y;
      points[i * 5 + 2] = opoints.at<OpointType>(i).x;
      points[i * 5 + 3] = opoints.at<OpointType>(i).y;
      points[i * 5 + 4] = opoints.at<OpointType>(i).z;
    }
    // Fill vectors with unused values for p3p case
    for (int i = npoints; i < 4; i++) {
      for (int j = 0; j < 5; j++) {
        points[i * 5 + j] = 0;
      }
    }
  }
  void init_inverse_parameters();

  //求解光心到各点的距离
  // Reference : X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution
  // Classification for the Perspective-Three-Point Problem"
  int solve_for_lengths(double lengths[4][3], double distances[3],
                        double cosines[3]);

  //求解光心到各点的距离
  //按照公式自己推导过程进行求解
  int my_solve_for_lengths(double lengths[4][3], double distances[3],
                           double cosines[3]);

  bool align(double M_start[3][3], double X0, double Y0, double Z0, double X1,
             double Y1, double Z1, double X2, double Y2, double Z2,
             double R[3][3], double T[3]);

  bool jacobi_4x4(double *A, double *D, double *U);

  double fx, fy, cx, cy;
  double inv_fx, inv_fy, cx_fx, cy_fy;
};

//多项式求解
//系数按照多项式的幂降序排列
int solve_deg2(double a, double b, double c, double &x1, double &x2);

int solve_deg3(double a, double b, double c, double d, double &x0, double &x1,
               double &x2);

int solve_deg4(double a, double b, double c, double d, double e, double &x0,
               double &x1, double &x2, double &x3);

//封装P3P为函数,保持和OpenCV一致

/** @brief Finds an object pose from 3 3D-2D point correspondences.

@param objectPoints Array of object points in the object coordinate space, 3x3
1-channel or 1x3/3x1 3-channel. vector\<Point3f\> can be also passed here.
@param imagePoints Array of corresponding image points, 3x2 1-channel or 1x3/3x1
2-channel. vector\<Point2f\> can be also passed here.
@param cameraMatrix Input camera matrix \f$A =
\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\f$ .
@param distCoeffs Input vector of distortion coefficients
\f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x,
\tau_y]]]])\f$ of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the
zero distortion coefficients are assumed.
@param rvecs Output rotation vectors (see @ref Rodrigues ) that, together with
tvecs, brings points from the model coordinate system to the camera coordinate
system. A P3P problem has up to 4 solutions.
@param tvecs Output translation vectors.
@param flags Method for solving a P3P problem:
-   **SOLVEPNP_P3P** Method is based on the paper of X.S. Gao, X.-R. Hou, J.
Tang, H.-F. Chang "Complete Solution Classification for the
Perspective-Three-Point Problem" (@cite gao2003complete).
-   **SOLVEPNP_AP3P** Method is based on the paper of T. Ke and S. Roumeliotis.
"An Efficient Algebraic Solution to the Perspective-Three-Point Problem" (@cite
Ke17).

The function estimates the object pose given 3 object points, their
corresponding image projections, as well as the camera matrix and the distortion
coefficients.

@note
The solutions are sorted by reprojection errors (lowest to highest).
 */
int SolveP3P(cv::InputArray objectPoints, cv::InputArray imagePoints,
             cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
             cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs,
             int flags);

#endif // OPENCVDEMO_P3P_H
