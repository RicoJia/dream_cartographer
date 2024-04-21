// g++ -std=c++17 2dlocalizer.cpp -O2 -pthread -lyaml-cpp -I /usr/include/eigen3
// `pkg-config --cflags --libs opencv4` && ./a.out map499.png 4 0
#include "2dlocalizer.hpp"
#include "ThreadPool.h"
#include <asm-generic/errno.h>
#include <chrono>
#include <future>
#include <iostream>

int main(int argc, char *argv[]) {
  InputArgs input_args = parse_input_args(argc, argv);
  std::cout << input_args << std::endl;
  auto laser_scans = read_laser_scans_from_csv("data/laser_scans.csv");
  print_scans(laser_scans);

  auto original_map_img = load_map_image(input_args.map_image_filename);
  auto map_img = map_image_preprocessing(original_map_img);
  auto metadata = load_map_metadata(input_args.map_metadata_filename, map_img);
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "metadata: " << metadata << std::endl;
  Eigen::Vector2i origin_xy_pixel =
      (-metadata.origin.head<2>() / metadata.resolution)
          .unaryExpr(RoundToInt());
  // visualize_map(map_img);
  auto trial_scan_msg = laser_scans.at(input_args.trial_laser_scan_id);
  auto search_thetas = arange(0.0, 2 * M_PI, input_args.num_angular_seg);
  auto bearings = arange(0.0, 2 * M_PI, trial_scan_msg.size());
  std::cout << "origin_xy_pixel: "
            << origin_xy_pixel.format(print_in_one_line_format) << std::endl;
  std::cout << "trian scan msg size: " << trial_scan_msg.size() << std::endl;
  // TODO
  // std::cout<<"search_thetas:
  // "<<search_thetas.format(print_in_one_line_format)<<std::endl;
  // std::cout<<"bearings
  // "<<bearings.format(print_in_one_line_format)<<std::endl;

  // Optimization
  auto p_hits_for_all_thetas = get_laser_endbeam_relative_for_all_thetas(
      search_thetas, bearings, trial_scan_msg, metadata.laser_max_range,
      metadata.resolution);
  // this is still relative to the body frame
  auto p_hits_for_all_thetas_matrix_coords = map_pixel_to_matrix_p_hits(
      p_hits_for_all_thetas, origin_xy_pixel, metadata.height);
  auto origin_in_matrix_coords =
      map_pixel_to_matrix_coords({0.0, 0.0}, origin_xy_pixel, metadata.height);
  std::vector<cv::Mat> p_hit_images_all_thetas;
  std::vector<Eigen::Vector2i> relative_robot_poses_in_mat;
  generate_smallest_matrices_for_all_thetas(
      p_hits_for_all_thetas_matrix_coords, origin_in_matrix_coords,
      p_hit_images_all_thetas, relative_robot_poses_in_mat);

  std::vector<cv::Mat> p_hits_for_all_thetas_image_gradients;
  std::transform(p_hit_images_all_thetas.begin(), p_hit_images_all_thetas.end(),
                 std::back_inserter(p_hits_for_all_thetas_image_gradients),
                 get_image_gradient);

  ThreadPool pool(16);
  std::vector<std::future<std::tuple<double, Eigen::Vector2i, unsigned int>>>
      futures;
  for (unsigned int i = 0; i < search_thetas.size(); i++) {
    auto fut = pool.enqueue([i, &map_img,
                             &p_hits_for_all_thetas_image_gradients,
                             &relative_robot_poses_in_mat] {
      return find_score(map_img, i, p_hits_for_all_thetas_image_gradients.at(i),
                        relative_robot_poses_in_mat.at(i));
    });
    futures.push_back(std::move(fut));
  }

  double best_score = 0.0;
  Eigen::Vector2i best_loc;
  unsigned int best_theta_id = 0;
  for (auto &fut : futures) {
    auto [max_val, max_loc, theta_id] = fut.get();
    if (max_val > best_score) {
      best_score = max_val;
      best_loc = max_loc;
      best_theta_id = theta_id;
    }
  }

  auto end = std::chrono::high_resolution_clock::now();

  auto best_loc_map_pixel =
      matrix_coords_to_map_pixel(best_loc, origin_xy_pixel, metadata.height);

  // Final Visualizations
  std::cout << "best_score: " << best_score
            << " best_loc: " << best_loc.format(print_in_one_line_format)
            << " best_loc_map_pixel: " << best_loc_map_pixel
            << ", best_theta_id: " << best_theta_id << std::endl;
  std::cout << "elapsed time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
  auto annoated_map_img = add_laser_scan_and_origin_to_map(
      original_map_img, best_loc_map_pixel, best_theta_id,
      p_hits_for_all_thetas.at(best_theta_id), origin_xy_pixel,
      metadata.height);
  // for (const auto& i: p_hits_for_all_thetas_image_gradients)
  // visualize_map(i);
  visualize_map(annoated_map_img);
  return 0;
}