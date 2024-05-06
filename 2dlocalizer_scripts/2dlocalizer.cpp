// g++ -std=c++17 2dlocalizer.cpp -O2 -pthread -lyaml-cpp -I /usr/include/eigen3
// `pkg-config --cflags --libs opencv4`
// ./a.out bird_world.pgm 256 200 cpu

// On GPU: ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2
// apt update && apt install libyaml-cpp-dev && apt-get install libeigen3-dev
//  g++ -std=c++17 2dlocalizer.cpp -O2 -pthread -lyaml-cpp -I
//  /usr/include/eigen3 `pkg-config --cflags --libs opencv4`
// Doc: Transforms used in this program:
// https://github.com/RicoJia/The-Dream-Robot/assets/39393023/9e8e8784-c5c4-431c-ad39-1c93fd9bcda5

// TODO: 1. free / occupancy thresholding
#include "2dlocalizer.hpp"
#include "ThreadPool.h"
#include <algorithm>
#include <asm-generic/errno.h>
#include <chrono>
#include <future>
#include <iostream>
#include <iterator>
#include <tuple>
#include <vector>

// Must be lower than 1
const double CORRECTION_PADDING_FACTOR = 0.0;

void preprocessing_and_load_data(const cv::Mat &original_map_img,
                                 const InputArgs &input_args,
                                 MapMetadata &metadata, cv::Mat &return_map_img,
                                 Eigen::Vector2i &origin_xy_pixel) {
  return_map_img = downsize_image(original_map_img, input_args.downsize_factor);
  return_map_img = map_image_preprocessing(return_map_img);
  metadata =
      load_map_metadata(input_args.map_metadata_filename, return_map_img);
  std::cout << "metadata: " << metadata << std::endl;
  /** Prepping data*/
  origin_xy_pixel = (-metadata.origin.head<2>() / metadata.resolution)
                        .unaryExpr(RoundToInt());
  origin_xy_pixel =
      downsize_vector2i(origin_xy_pixel, input_args.downsize_factor);
}

void get_p_hits(
    const Eigen::VectorXd &search_thetas, const Eigen::VectorXd &bearings,
    const std::vector<double> &chosen_scan_msg, const MapMetadata &metadata,
    const Eigen::Vector2i &origin_xy_pixel,
    std::vector<std::vector<Eigen::Vector2i>> &p_hits_for_all_thetas,
    std::vector<cv::Mat> &masks_for_all_thetas,
    std::vector<Eigen::Vector2i> &relative_robot_poses_in_mat) {
  // [theta1[(x,y)...], theta2 [...]]
  p_hits_for_all_thetas = get_laser_endbeam_relative_for_all_thetas(
      search_thetas, bearings, chosen_scan_msg, metadata.laser_max_range,
      metadata.resolution);

  // this is still relative to the body frame
  auto p_hits_for_all_thetas_matrix_coords = map_pixel_to_matrix_p_hits(
      p_hits_for_all_thetas, origin_xy_pixel, metadata.height);
  auto origin_in_matrix_coords =
      map_pixel_to_matrix_coords({0.0, 0.0}, origin_xy_pixel, metadata.height);
  std::vector<cv::Mat> p_hit_images_all_thetas;
  generate_smallest_matrices_for_all_thetas(
      p_hits_for_all_thetas_matrix_coords, origin_in_matrix_coords,
      p_hit_images_all_thetas, relative_robot_poses_in_mat);

  std::transform(p_hit_images_all_thetas.begin(), p_hit_images_all_thetas.end(),
                 std::back_inserter(masks_for_all_thetas), get_image_gradient);
}

cv::Rect get_roi_window(const Eigen::Vector2i &top_left_pixel,
                        const cv::Mat &mask, const cv::Mat &map_image) {
  // x this is actually columns in image, y is rows
  Eigen::Vector2i padding =
      (CORRECTION_PADDING_FACTOR * Eigen::Vector2d(mask.cols, mask.rows))
          .unaryExpr(RoundToInt());
  // Eigen::Vector2i bottom_right_pixel = top_left_pixel +
  // Eigen::Vector2i(mask.cols, mask.rows);
  Eigen::Vector2i top_left_roi_pixel = top_left_pixel - padding;
  // Eigen::Vector2i bottom_right_roi_pixel = bottom_right_pixel + padding;

  // bounds check
  top_left_roi_pixel[0] = std::max(0, top_left_roi_pixel[0]);
  top_left_roi_pixel[1] = std::max(0, top_left_roi_pixel[1]);
  // bottom_right_roi_pixel[0] = std::min(bottom_right_roi_pixel[0],
  // map_image.cols-1); bottom_right_roi_pixel[1] =
  // std::min(bottom_right_roi_pixel[1], map_image.rows-1);

  Eigen::Vector2i roi_size = padding + Eigen::Vector2i(mask.cols, mask.rows);
  auto roi = cv::Rect(top_left_roi_pixel[0], top_left_roi_pixel[1], roi_size[0],
                      roi_size[1]);
  return roi;
}

std::tuple<std::vector<Eigen::Vector2i>, std::vector<unsigned int>,
           std::vector<double>>
process_futures(
    std::vector<std::future<std::tuple<double, Eigen::Vector2i, unsigned int>>>
        &futures,
    const int &downsize_factor) {
  std::vector<Eigen::Vector2i> candidate_image_coords;
  std::vector<unsigned int> theta_ids;
  std::vector<double> scores;
  for (auto &fut : futures) {
    auto [max_val, max_loc, theta_id] = fut.get();
    candidate_image_coords.push_back(
        recover_size_vector2i(max_loc, downsize_factor));
    theta_ids.push_back(theta_id);
    scores.push_back(max_val);
  }
  futures.clear();
  return std::make_tuple(candidate_image_coords, theta_ids, scores);
}

// Returns candidates in map pixels, theta_ids, and scores
std::tuple<std::vector<Eigen::Vector2i>, std::vector<unsigned int>,
           std::vector<double>>
post_processing(
    std::vector<std::future<std::tuple<double, Eigen::Vector2i, unsigned int>>>
        &futures,
    const int &downsize_factor, const std::vector<cv::Rect> &roi_windows,
    const std::vector<Eigen::Vector2i> &relative_robot_poses_in_mat,
    const Eigen::Vector2i &origin_xy_pixel, const MapMetadata &metadata) {
  auto [candidate_image_coords, theta_ids, scores] =
      process_futures(futures, downsize_factor);

  std::vector<Eigen::Vector2i> candidate_map_coords(
      candidate_image_coords.size());
  for (unsigned int i = 0; i < candidate_image_coords.size(); ++i) {
    Eigen::Vector2i top_left_pixel_roi(roi_windows.at(i).x,
                                       roi_windows.at(i).y);
    candidate_image_coords.at(i) += top_left_pixel_roi;
    auto candidate_matrix_coord =
        image_coords_to_matrix_coords(candidate_image_coords.at(i));
    candidate_matrix_coord += relative_robot_poses_in_mat.at(i);
    candidate_map_coords.at(i) = matrix_coords_to_map_pixel(
        candidate_matrix_coord, origin_xy_pixel, metadata.height);
  }
  return std::make_tuple(candidate_map_coords, theta_ids, scores);
}

void show_results(
    const std::vector<Eigen::Vector2i> &candidate_map_coords,
    const std::vector<unsigned int> &theta_ids,
    const std::vector<double> &scores, const Eigen::Vector2i &origin_xy_pixel,
    const MapMetadata &metadata,
    const std::vector<std::vector<Eigen::Vector2i>> &p_hits_for_all_thetas,
    const InputArgs &input_args, const cv::Mat &original_map_img

) {

  auto max_iterator = std::max_element(scores.begin(), scores.end());
  auto max_i = std::distance(scores.begin(), max_iterator);
  auto best_theta_id = theta_ids.at(max_i);
  auto best_score = scores.at(max_i);
  auto best_loc_map_pixel = candidate_map_coords.at(max_i);
  auto best_loc_image_pixel = map_pixel_to_image_coords(
      best_loc_map_pixel, origin_xy_pixel, metadata.height);

  std::cout << "best_score: " << best_score << ", best_loc_image_pixel: "
            << best_loc_image_pixel.format(print_in_one_line_format)
            << ", best_theta_id: " << best_theta_id << std::endl;
  auto annoated_map_img = add_laser_scan_and_origin_to_map(
      original_map_img, origin_xy_pixel, metadata.height, best_loc_map_pixel,
      p_hits_for_all_thetas.at(best_theta_id));
  visualize_map(annoated_map_img);

  // Showing candidate locations
  // for (unsigned int i = 0; i < theta_ids.size(); ++i){
  //     const auto& loc_map_pixel = candidate_map_coords.at(i);
  //     auto theta_id = theta_ids.at(i);
  //     auto loc_img_pixel = map_pixel_to_image_coords(
  //         loc_map_pixel, origin_xy_pixel, metadata.height);
  //     auto annoated_map_img = add_laser_scan_and_origin_to_map(
  //         original_map_img, origin_xy_pixel, metadata.height, loc_map_pixel,
  //         p_hits_for_all_thetas.at(theta_id));
  //     visualize_map(annoated_map_img);
  // }
}

int main(int argc, char *argv[]) {
  // 0. Read inputs
  InputArgs input_args = parse_input_args(argc, argv);
  auto laser_scans = read_laser_scans_from_csv("data/laser_scans.csv");
  auto chosen_scan_msg = laser_scans.at(input_args.trial_laser_scan_id);
  auto search_thetas = arange(0.0, 2 * M_PI, input_args.num_angular_seg);
  auto bearings = arange(0.0, 2 * M_PI, chosen_scan_msg.size());
  auto original_map_img = load_map_image(input_args.map_image_filename);
  print_scans(laser_scans);
  std::cout << input_args << std::endl;

  ////////////////////////////////////////////////////////////////
  /** Start the timer*/
  // 1. Set up common variables between runs
  auto start = std::chrono::high_resolution_clock::now();
  ThreadPool pool(16);
  std::vector<int> downsize_factors{input_args.downsize_factor, 1};
  // We want to find coarse estimates on downsized images, then come back to
  // the original image
  std::vector<std::future<std::tuple<double, Eigen::Vector2i, unsigned int>>>
      futures;
  std::vector<cv::Rect> roi_windows(search_thetas.size(), cv::Rect(0, 0, 0, 0));
  std::vector<Eigen::Vector2i> relative_robot_poses_in_mat;
  Eigen::Vector2i origin_xy_pixel;
  MapMetadata metadata;
  std::vector<std::vector<Eigen::Vector2i>> p_hits_for_all_thetas;

  for (unsigned int downsize_i = 0; downsize_i < downsize_factors.size();
       ++downsize_i) {
    /** 2. Get temporary data ready*/
    input_args.downsize_factor = downsize_factors.at(downsize_i);
    cv::Mat map_img;
    std::vector<cv::Mat> masks_for_all_thetas;
    preprocessing_and_load_data(original_map_img, input_args, metadata, map_img,
                                origin_xy_pixel);
    auto downsized_scan_msg =
        downsize_laser_scan_msg(chosen_scan_msg, input_args.downsize_factor);
    get_p_hits(search_thetas, bearings, downsized_scan_msg, metadata,
               origin_xy_pixel, p_hits_for_all_thetas, masks_for_all_thetas,
               relative_robot_poses_in_mat);

    /** 3. Process futures in the thread pool and get best candidates, which
     * have been running in the background */
    /** In the first iteration, futures is empty, so this won't run */
    // std::vector<Eigen::Vector2i> candidate_image_coords;
    // std::vector<unsigned int> theta_ids;
    // std::vector<double> scores;
    // if (downsize_i > 1){
    // }

    /** 4. Process futures in the thread pool and get best candidates, which
     * have been running in the background, then Get roi and masks ready for
     * scoring. */
    // 2nd run above (candidate_image_coords is empty, so wouldn't run in the
    // first iteration)
    std::vector<cv::Mat> rois(search_thetas.size(), map_img);
    if (downsize_i >= 1) {
      auto [candidate_image_coords, theta_ids, scores] =
          process_futures(futures, downsize_factors.at(downsize_i - 1));
      for (unsigned int i = 0; i < candidate_image_coords.size(); i++) {
        const auto theta_id = theta_ids.at(i);
        const auto top_left_pixel = candidate_image_coords.at(i);
        roi_windows.at(i) =
            get_roi_window(top_left_pixel, masks_for_all_thetas.at(i), map_img);
        rois.at(i) = map_img(roi_windows.at(i));
      }
    } else {
      roi_windows = std::vector<cv::Rect>(
          search_thetas.size(), cv::Rect(0, 0, map_img.cols, map_img.rows));
    }

    /** 5. Scoring in Thread pool */
    for (unsigned int i = 0; i < search_thetas.size(); i++) {
      auto fut = pool.enqueue([i, rois, masks_for_all_thetas,
                               relative_robot_poses_in_mat, input_args] {
        return find_score(rois.at(i), i, masks_for_all_thetas.at(i),
                          relative_robot_poses_in_mat.at(i),
                          input_args.use_gpu);
      });
      futures.push_back(std::move(fut));
    }
  }
  /** 6. Post processing */
  auto [candidate_map_coords, theta_ids, scores] =
      post_processing(futures, downsize_factors.back(), roi_windows,
                      relative_robot_poses_in_mat, origin_xy_pixel, metadata);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "elapsed time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;

  show_results(candidate_map_coords, theta_ids, scores, origin_xy_pixel,
               metadata, p_hits_for_all_thetas, input_args, original_map_img);

  return 0;
}