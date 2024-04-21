#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>

#include "ThreadPool.h"

// ----------------------------------------------------------------
// Utils
// ----------------------------------------------------------------
Eigen::IOFormat print_in_one_line_format(4, 0, ", ", ", ", "", "", "", "");
struct RoundToInt {
  // Functor to round a double to int
  int operator()(double coeff) const {
    return static_cast<int>(std::round(coeff));
  }
};

struct InputArgs {
  std::string map_image_filename;
  std::string map_metadata_filename;
  int num_angular_seg;
  int trial_laser_scan_id;
  bool use_gpu;
  friend std::ostream &operator<<(std::ostream &os, const InputArgs &args) {
    os << " map_image_filename: " << args.map_image_filename << ", ";
    os << " map_metadata_filename: " << args.map_metadata_filename;
    os << " num_angular_seg: " << args.num_angular_seg;
    os << " trial_laser_scan_id: " << args.trial_laser_scan_id;
    os << std::endl;
    return os;
  }
};

InputArgs parse_input_args(int argc, char **argv) {
  if (argc != 5)
    throw std::runtime_error("usage: 2dlocalizer <map_image_filename> "
                             "<num_angular_seg> <trial_laser_scan_id> <cpu_or_gpu>");
  InputArgs args;
  args.map_image_filename = argv[1];
  std::regex rgx(
      "(.+)\\.(png|pgm)$"); // Regex to capture the name before ".pgm"
  std::smatch matches;
  if (std::regex_search(args.map_image_filename, matches, rgx) &&
      matches.size() > 1) {
    args.map_metadata_filename = matches[1].str() + ".yaml";
  } else {
    throw std::runtime_error("Need '.' in filename: " +
                             args.map_image_filename);
  }
  args.num_angular_seg = std::atoi(argv[2]);
  args.trial_laser_scan_id = std::atoi(argv[3]);
  args.use_gpu = std::strcmp(argv[4], "gpu") == 0;
  return args;
}

inline std::vector<std::vector<double>>
read_laser_scans_from_csv(const std::string &filename) {
  std::ifstream file(filename.c_str());
  if (!file.is_open())
    throw std::runtime_error("Could not open file: " + filename);

  std::string line;
  std::vector<std::vector<double>> result;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> row;
    // csv file can contain "inf". Replacing it with
    // std::numeric_limits<double>::infinity()
    while (std::getline(ss, value, ',')) {
      if (value == "inf" || value == "-inf") {
        row.push_back(std::numeric_limits<double>::infinity());
      } else {
        row.push_back(std::stod(value));
      }
    }
    result.push_back(row);
  }
  file.close();
  return result;
}

inline void print_scans(const std::vector<std::vector<double>> &laser_scans) {
  // for(const auto& scan : laser_scans){
  //     for(const auto& scan_value : scan) std::cout<<scan_value<<", ";
  //     std::cout<<std::endl;
  // }
  std::cout << "Number of scans: " << laser_scans.size() << std::endl
            << std::endl;
}

struct MapMetadata {
  double resolution;
  Eigen::Vector3d origin;
  double width, height;
  double laser_max_range;
  friend std::ostream &operator<<(std::ostream &os,
                                  const MapMetadata &metadata) {
    os << "--------------------------------" << std::endl;
    os << "resolution: " << metadata.resolution << std::endl;
    os << "origin: "
       << "[" << metadata.origin[0] << ", " << metadata.origin[1] << ", "
       << metadata.origin[2] << "]" << std::endl;
    os << "width: " << metadata.width << std::endl;
    os << "height: " << metadata.height << std::endl;
    return os;
  }
};

inline MapMetadata load_map_metadata(const std::string &filename,
                                     const cv::Mat &map_image) {
  YAML::Node config = YAML::LoadFile(filename);
  auto metadata = MapMetadata();
  metadata.resolution = config["resolution"].as<double>();
  std::vector<double> temp;
  for (const auto &origin : config["origin"])
    temp.push_back(origin.as<double>());
  metadata.origin = Eigen::Map<Eigen::Vector3d>(temp.data(), temp.size());
  metadata.width = map_image.cols;
  metadata.height = map_image.rows;
  // TODO: to make this a parameter
  metadata.laser_max_range = 10.0;
  return metadata;
}

inline cv::Mat load_map_image(const std::string &filename) {
  auto img = cv::imread(filename, cv::IMREAD_COLOR);
  if (img.empty())
    throw std::runtime_error("Could not load image: " + filename);
  return img;
}

// ----------------------------------------------------------------
// Coordinate Transforms
// ----------------------------------------------------------------

// First order transforms
inline Eigen::Vector2i
map_to_shifted_map(const Eigen::Vector2i &map_point,
                   const Eigen::Vector2i &origin_xy_pixel) {
  return map_point + origin_xy_pixel;
}

inline Eigen::Vector2i
shifted_map_to_map(const Eigen::Vector2i &shifted_map_point,
                   const Eigen::Vector2i &origin_xy_pixel) {
  return shifted_map_point - origin_xy_pixel;
}

inline Eigen::Vector2i
shifted_map_to_matrix_coords(const Eigen::Vector2i &shifted_map_point,
                             const double &img_height) {
  return Eigen::Vector2i(img_height - shifted_map_point[1],
                         shifted_map_point[0]);
}

inline Eigen::Vector2i
matrix_coords_to_shifted_map(const Eigen::Vector2i &matrix_coords,
                             const double &img_height) {
  return Eigen::Vector2i(matrix_coords[1], img_height - matrix_coords[0]);
}

inline Eigen::Vector2i
matrix_coords_to_image_coords(const Eigen::Vector2i &matrix_coords,
                              const double &img_height) {
  // have to swap x and y for image visualization
  return Eigen::Vector2i(matrix_coords[1], matrix_coords[0]);
}

// Second order transforms
inline Eigen::Vector2i
map_pixel_to_matrix_coords(const Eigen::Vector2i &map_point,
                           const Eigen::Vector2i &origin_xy_pixel,
                           const double &img_height) {
  Eigen::Vector2i shifted_map_point =
      map_to_shifted_map(map_point, origin_xy_pixel);
  Eigen::Vector2i matrix_coords =
      shifted_map_to_matrix_coords(shifted_map_point, img_height);
  return matrix_coords;
}

inline Eigen::Vector2i
matrix_coords_to_map_pixel(const Eigen::Vector2i &matrix_coords,
                           const Eigen::Vector2i &origin_xy_pixel,
                           const double &img_height) {
  auto shifted_map_coords =
      matrix_coords_to_shifted_map(matrix_coords, img_height);
  auto map_pixel = shifted_map_to_map(shifted_map_coords, origin_xy_pixel);
  return map_pixel;
}

inline Eigen::Vector2i
shifted_map_to_image_pixel(const Eigen::Vector2i &shifted_map_coords,
                           const double &img_height) {
  // have to swap x and y for image visualization
  return Eigen::Vector2i(shifted_map_coords[0],
                         img_height - shifted_map_coords[1]);
}

// Third order transforms
inline Eigen::Vector2i
map_pixel_to_image_coords(const Eigen::Vector2i &map_point,
                          const Eigen::Vector2i &origin_xy_pixel,
                          const double &img_height) {
  // have to swap x and y for image visualization
  auto matrix_coords =
      map_pixel_to_matrix_coords(map_point, origin_xy_pixel, img_height);
  return matrix_coords_to_image_coords(matrix_coords, img_height);
}

// ----------------------------------------------------------------
// Business Logic
// ----------------------------------------------------------------

inline cv::Mat get_image_gradient(const cv::Mat &binary_image) {
  cv::Mat gradients;
  cv::GaussianBlur(binary_image, gradients, cv::Size(3, 3), 0, 0);
  cv::Laplacian(gradients, gradients, CV_8UC1);
  return gradients;
}

inline cv::Mat get_binary_image(const cv::Mat &image) {
  cv::Mat new_image;
  double thresh = 0;
  double max_val = 255;
  cv::threshold(image, new_image, thresh, max_val, cv::THRESH_BINARY);
  return new_image;
}

inline cv::Mat map_image_preprocessing(const cv::Mat &map_image) {
  // to gray
  cv::Mat new_map_image;
  cv::cvtColor(map_image, new_map_image, cv::COLOR_RGBA2GRAY);
  new_map_image = get_binary_image(new_map_image);
  new_map_image = get_image_gradient(new_map_image);
  return new_map_image;
}

inline Eigen::VectorXd arange(const double &start, const double &end,
                              const int &num) {
  const double step = (end - start) / num;
  Eigen::VectorXd ret(num);
  for (int i = 0; i < num; i++)
    ret(i) = start + i * step;
  return ret;
};

inline std::vector<std::vector<Eigen::Vector2i>>
get_laser_endbeam_relative_for_all_thetas(
    Eigen::Ref<Eigen::VectorXd> search_thetas,
    Eigen::Ref<Eigen::VectorXd> bearings, const std::vector<double> &scan_msg,
    const double &laser_max_range, const double &resolution) {
  // return array doesn't have inf. And they are in pixelized map frame
  if (scan_msg.size() != bearings.size())
    throw std::runtime_error(
        "number of bearings and scan_msg should be the same");
  // [theta1[(x,y)...], theta2 [...]]
  std::vector<std::vector<Eigen::Vector2i>> p_hits_for_all_thetas(
      search_thetas.size());
  for (unsigned int j = 0; j < search_thetas.size(); j++) {
    const double theta = search_thetas[j];
    for (unsigned int i = 0; i < scan_msg.size(); i++) {
      const auto &d = scan_msg[i];
      const auto &bearing = bearings[i];
      if (d <= laser_max_range) {
        p_hits_for_all_thetas[j].push_back(
            // x, y
            {RoundToInt()(d * std::cos(bearing + theta) / resolution),
             RoundToInt()(d * std::sin(bearing + theta) / resolution)});
      }
    }
  }
  return p_hits_for_all_thetas;
}

inline std::vector<std::vector<Eigen::Vector2i>> map_pixel_to_matrix_p_hits(
    std::vector<std::vector<Eigen::Vector2i>> p_hits_for_all_thetas,
    const Eigen::Vector2i &origin_xy_pixel, const double &img_height) {
  std::vector<std::vector<Eigen::Vector2i>>
      p_hits_for_all_thetas_in_matrix_coords(p_hits_for_all_thetas.size());
  for (unsigned int i = 0; i < p_hits_for_all_thetas.size(); i++) {
    for (const auto &p_hits : p_hits_for_all_thetas[i]) {
      p_hits_for_all_thetas_in_matrix_coords[i].push_back(
          map_pixel_to_matrix_coords(p_hits, origin_xy_pixel, img_height));
    }
  }
  return p_hits_for_all_thetas_in_matrix_coords;
}

inline void generate_smallest_matrices_for_all_thetas(
    const std::vector<std::vector<Eigen::Vector2i>> &p_hits_for_all_thetas,
    const Eigen::Vector2i &orig_in_matrix_coord,
    std::vector<cv::Mat> &matrices_for_al_thetas,
    std::vector<Eigen::Vector2i> &relative_robot_poses_in_mat) {
  matrices_for_al_thetas = std::vector<cv::Mat>(p_hits_for_all_thetas.size());
  relative_robot_poses_in_mat =
      std::vector<Eigen::Vector2i>(p_hits_for_all_thetas.size());
  for (unsigned int i = 0; i < p_hits_for_all_thetas.size(); i++) {
    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = 0;
    int max_y = 0;
    const auto &p_hits = p_hits_for_all_thetas[i];
    for (const auto &p_hit : p_hits) {
      min_x = std::min(min_x, p_hit[0]);
      min_y = std::min(min_y, p_hit[1]);
      max_x = std::max(max_x, p_hit[0]);
      max_y = std::max(max_y, p_hit[1]);
    }
    matrices_for_al_thetas[i] =
        255 * cv::Mat::ones(max_x - min_x + 1, max_y - min_y + 1, CV_8UC1);
    for (const auto &p_hit : p_hits) {
      matrices_for_al_thetas[i].at<unsigned char>(p_hit[0] - min_x,
                                                  p_hit[1] - min_y) = 0;
    }
    relative_robot_poses_in_mat[i] =
        orig_in_matrix_coord - Eigen::Vector2i(min_x, min_y);
  }
}

/**
 * @brief
 *
 * @param binary_map_image : map image in binary form (0, 255)
 * @param theta_id : id of the theta
 * @param templ : template image
 * @param relative_robot_pose : origin relative to the top left corner of the
 * p_hit_image
 * @return std::tuple<double, Eigen::Vector2i, unsigned int> : best estimate of
 * robot's (x,y) in matrix coords
 */
inline std::tuple<double, Eigen::Vector2i, unsigned int>
find_score(const cv::Mat &binary_map_image, const unsigned int &theta_id,
           const cv::Mat &templ,
           const Eigen::Vector2i &relative_robot_pose_in_mat,
           const bool& use_gpu) {
  int result_col_num = binary_map_image.cols - templ.cols + 1;
  int result_row_num = binary_map_image.rows - templ.rows + 1;
    cv::Mat res;
    if (use_gpu) {
        cv::cuda::GpuMat d_img(binary_map_image);
        cv::cuda::GpuMat d_templ(templ);
        cv::cuda::GpuMat d_result;
        cv::Ptr<cv::cuda::TemplateMatching> matcher = cv::cuda::createTemplateMatching(binary_map_image.type(), cv::TM_CCOEFF);
        matcher->match(d_img, d_templ, d_result);
        d_result.download(result);
    }else{
        res = cv::Mat(result_row_num, result_col_num, CV_32FC1);
        cv::matchTemplate(binary_map_image, templ, res, cv::TM_CCOEFF);
    }

  // cv::normalize(res, res, 0, 1, cv::NORM_MINMAX, -1);
  double min_val, max_val;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(res, &min_val, &max_val, &min_loc, &max_loc);
  // Important: cv2.minMaxLoc returns the reversed  matrix index [y,x]
  Eigen::Vector2i max_loc_vec(max_loc.y, max_loc.x);
  // TODO
  //   std::cout<<"raw max_loc_vec: "<<max_loc_vec<<std::endl;
  max_loc_vec += relative_robot_pose_in_mat;
  //   std::cout<<"relative max_loc_vec:
  //   "<<max_loc_vec.format(print_in_one_line_format)<<std::endl;
  return std::make_tuple(max_val, max_loc_vec, theta_id);
}

inline cv::Point get_cv_point(const Eigen::Vector2i &point) {
  return cv::Point(point[0], point[1]);
}

inline void add_point_to_image(const cv::Point &point, cv::Mat &img,
                               const std::string &text) {
  // Colors
  cv::Scalar redColor(0, 0, 255);          // BGR format for red
  cv::circle(img, point, 2, redColor, -1); // -1 fill the circle
  if (text != "") {
    cv::putText(img, text, cv::Point(point.x + 10, point.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, redColor, 1);
  }
}

inline cv::Mat add_laser_scan_and_origin_to_map(
    const cv::Mat &map_image, const Eigen::Vector2i &best_loc_map_pixel,
    const unsigned int &theta_id,
    const std::vector<Eigen::Vector2i> &p_hits_relative_map_pixel,
    const Eigen::Vector2i &origin_px, const double &img_height) {
  cv::Mat new_map_image = map_image.clone();
  auto origin_px_map = shifted_map_to_image_pixel(origin_px, img_height);
  auto origin_point = get_cv_point(origin_px_map);
  add_point_to_image(origin_point, new_map_image, "origin");

  auto best_loc_image_pose =
      map_pixel_to_image_coords(best_loc_map_pixel, origin_px, img_height);
  auto best_loc_point = get_cv_point(best_loc_image_pose);
  // TODO
  std::cout << "best_loc_point: " << best_loc_point << std::endl;
  add_point_to_image(best_loc_point, new_map_image, "robot");

  for (const auto &p_hit_relative_map : p_hits_relative_map_pixel) {
    auto p_hit_absolute_map = best_loc_map_pixel + p_hit_relative_map;
    auto p_hit_img =
        map_pixel_to_image_coords(p_hit_absolute_map, origin_px, img_height);
    auto p_hit_point = get_cv_point(p_hit_img);
    add_point_to_image(p_hit_point, new_map_image, "");
  }
  return new_map_image;
}

// if score > TEMPLATE_MATCHING_THRESHOLD:
//     max_loc = max_loc[::-1]
//     #TODO Remember to remove
//     print(f'Rico: max_loc raw: {max_loc}')
//     max_loc += relative_robot_poses_in_mat_theta
//     max_loc_map_pose = matrix_coords_to_image_coords(
//         np.array([max_loc]), origin_px, img_height
//     )
//     p_hits_for_theta_absolute = add_pose_to_relative_poses(
//         [p_hits_for_all_thetas[theta_id]], max_loc_map_pose
//     )
//     scores.append(
//         score
//     )
//     points.append(max_loc)
//     p_hits.append(p_hits_for_theta_absolute)
//     print("max_loc_map_pose", max_loc_map_pose, "score: ", score)
// return scores, points, p_hits

inline void visualize_map(const cv::Mat &map) {
  cv::imshow("Map Display", map);
  cv::waitKey(0);
}