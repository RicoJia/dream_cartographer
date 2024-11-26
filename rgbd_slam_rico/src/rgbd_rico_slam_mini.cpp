#include "rgbd_slam_rico/debug_utils.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_backend.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/io_utils.hpp"
#include "simple_robotics_cpp_utils/loguru.hpp"

using namespace RgbdSlamRico;

SLAMParams read_params(ros::NodeHandle nh) {
  int argc = 1; // argument count, the program's name itself
  char *argv[] = {(char *)("this_test_program"), NULL}; // argument vector
  SLAMParams slam_params;

  nh.getParam("bag_name", slam_params.bag_name);

  nh.getParam("image_skip_batch_num", slam_params.image_skip_batch_num);
  nh.getParam("image_num", slam_params.image_num);
  nh.getParam("test_with_optimization", slam_params.test_with_optimization);
  std::string pnp_method;
  nh.getParam("pnp_method", pnp_method);
  slam_params.pnp_method_enum = read_pnp_method(pnp_method);
  nh.getParam("downsample_point_cloud", slam_params.downsample_point_cloud);
  nh.getParam("voxel_size", slam_params.voxel_size);
  nh.getParam("nearby_vertex_check_num", slam_params.nearby_vertex_check_num);
  nh.getParam("random_vertex_check_num", slam_params.random_vertex_check_num);
  nh.getParam("robust_kernel_name", slam_params.robust_kernel_name);
  nh.getParam("min_ransac_feature_inliers",
              slam_params.min_ransac_feature_inliers);

  nh.getParam("min_interframe_translation_thre",
              slam_params.min_interframe_translation_thre);
  nh.getParam("min_interframe_rotation_thre",
              slam_params.min_interframe_rotation_thre);
  nh.getParam("max_interframe_rotation_thre",
              slam_params.max_interframe_rotation_thre);
  nh.getParam("max_interframe_translation_thre",
              slam_params.max_interframe_translation_thre);

  nh.getParam("use_ransac_for_pnp", slam_params.use_ransac_for_pnp);
  nh.getParam("do_ba_backend", slam_params.do_ba_backend);
  nh.getParam("min_depth", slam_params.min_depth);
  nh.getParam("max_depth", slam_params.max_depth);
  nh.getParam("verbose", slam_params.verbose);
  nh.getParam("visualize_frames", slam_params.visualize_frames);
  nh.getParam("pause_after_optimization", slam_params.pause_after_optimization);
  nh.getParam("initial_image_skip_num", slam_params.initial_image_skip_num);
  nh.getParam("save_pcd_file", slam_params.save_pcd_file);
  nh.getParam("filter_out_dark_pixels", slam_params.filter_out_dark_pixels);
  nh.getParam("inflation_pixels", slam_params.inflation_pixels);

  LOG_S(INFO) << "Done Reading Params";
  return slam_params;
}

int main(int argc, char *argv[]) {
  // Step 1: set up ros node, and parameters
  ros::init(argc, argv, "rgbd_rico_slam_mini");
  ros::NodeHandle nh;
  loguru::init(argc, argv);
  LOG_SCOPE_FUNCTION(INFO);
  auto slam_params = read_params(nh);

  // Step 2: bagparser, topics and camera info
  ros::Publisher poses_pub =
      nh.advertise<geometry_msgs::PoseArray>("optimized_poses", 1, true);
  ros::Publisher points_pub =
      nh.advertise<PointCloud>("optimized_points", 1, true);
  auto image_pub = ImagePub(nh, RGB_TOPIC);
  auto debug_added_keyframe_pub = ImagePub(nh, "/debug_added_keyframe");

  SimpleRoboticsRosUtils::BagParser bp(nh, PACKAGE_NAME, slam_params.bag_name);
  bp.fast_forward(RGB_TOPIC, slam_params.initial_image_skip_num);
  bp.fast_forward(DEPTH_TOPIC, slam_params.initial_image_skip_num);

  // Step 3:using namespace RgbdSlamRico;using namespace RgbdSlamRico;
  // initialize data structures for optimization
  HandyCameraInfo cam_info = load_camera_info(bp, CAMERA_INFO_TOPIC);
  PointCloud::Ptr point_cloud = initialize_point_cloud();

  std::vector<KeyFrameData> keyframes;
  keyframes.reserve(100);

  //  getting [initial_image_skip_num, initial_image_skip_num +
  //  image_skip_batch_num * image_num]
  for (unsigned int i = slam_params.initial_image_skip_num;
       i < slam_params.initial_image_skip_num + slam_params.image_num; i++) {
    if (!ros::ok())
      break;
    // Step 4: load images
    bp.fast_forward(RGB_TOPIC, slam_params.image_skip_batch_num);
    bp.fast_forward(DEPTH_TOPIC, slam_params.image_skip_batch_num);
    KeyFrameData current_keyframe{load_next_image_TUM(bp, RGB_TOPIC, true),
                                  load_next_image_TUM(bp, DEPTH_TOPIC, false),
                                  Eigen::Isometry3d::Identity(), i};
    if (current_keyframe.image.empty() ||
        current_keyframe.depth_image.empty()) {
      LOG_S(WARNING) << "At least one image topic is done. Exiting";
      break;
    }
    image_pub.publish(current_keyframe.image);
    // Step 5: detect ORB features
    auto orb_features = get_valid_orb_features(slam_params, current_keyframe);
    if (!orb_features.has_value())
      continue;
    current_keyframe.orb_res = std::move(orb_features.value());
    if (!filter_orb_result_with_valid_depths(current_keyframe.depth_image,
                                             slam_params,
                                             current_keyframe.orb_res))
      continue;
    // Step 6: Initialize
    if (keyframes.empty()) {
      initialize_global_optimizer(slam_params.verbose, current_keyframe);
      keyframes.emplace_back(std::move(current_keyframe));
      continue;
    }

    if (slam_params.pause_after_optimization) {
      const std::string msg =
          "Iteration " + std::to_string(i) + ", Please hit enter pause";
      SimpleRoboticsCppUtils::sleep_or_pause_on_keystroke(
          SimpleRoboticsCppUtils::to_color_msg(
              SimpleRoboticsCppUtils::ANSIStrings::RED, msg, true),
          1500);
    }

    if (slam_params.test_with_optimization) {
      auto frame1_to_frame2 =
          front_end(cam_info, slam_params, keyframes.back(), current_keyframe);
      if (!frame1_to_frame2.has_value())
        continue;
      add_single_frame_to_graph(keyframes.back(), current_keyframe,
                                frame1_to_frame2.value(), true,
                                slam_params.robust_kernel_name);

      if (slam_params.do_ba_backend) {
        add_edges_to_previous_vertices(current_keyframe, keyframes, cam_info,
                                       slam_params, EdgeAdditionMode::NEARBY);
      }
      // This pose is not used by global optimizer. This is for visualization
      // only
      current_keyframe.pose = keyframes.back().pose * frame1_to_frame2.value();
    }

    add_point_cloud(current_keyframe.pose, current_keyframe.image,
                    current_keyframe.depth_image, cam_info, point_cloud,
                    slam_params);
    // so current_keyframe is invalidated
    keyframes.emplace_back(std::move(current_keyframe));
    visualize_slam_results(keyframes, poses_pub, points_pub, point_cloud);
    if (slam_params.visualize_frames) {
      debug_added_keyframe_pub.publish(keyframes.back().image);
    }
  } // end for

  // Step 10: global BA optimization (backend)
  backend_optimize(keyframes);
  clear_pointcloud(point_cloud);
  std::cout << "Finished optimization. Adding point cloud now..."
            << ros::Time::now() << std::endl;
  add_point_cloud_multithreaded(keyframes, cam_info, point_cloud, slam_params);
  visualize_slam_results(keyframes, poses_pub, points_pub, point_cloud);
  std::cout << "Finished point cloud addition" << ros::Time::now() << std::endl;
  if (slam_params.save_pcd_file) {
    save_point_cloud_to_pcd(*point_cloud);
  }

  ros::spin();

  return 0;
}
