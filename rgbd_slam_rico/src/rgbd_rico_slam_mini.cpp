#include "rgbd_slam_rico/debug_utils.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_backend.hpp"
#include "rgbd_slam_rico/rgbd_rico_slam_frontend.hpp"
#include "simple_robotics_cpp_utils/io_utils.hpp"

using namespace RgbdSlamRico;
constexpr auto PACKAGE_NAME = "rgbd_slam_rico";
constexpr auto RGB_TOPIC = "/camera/rgb/image_color";
constexpr auto DEPTH_TOPIC = "/camera/depth/image";
constexpr auto CAMERA_INFO_TOPIC = "/camera/rgb/camera_info";

SLAMParams read_params(ros::NodeHandle nh) {
  SLAMParams slam_params;
  nh.getParam("image_skip_batch_num", slam_params.image_skip_batch_num);
  nh.getParam("image_num", slam_params.image_num);
  nh.getParam("test_with_optimization", slam_params.test_with_optimization);
  std::string pnp_method;
  nh.getParam("pnp_method", pnp_method);
  slam_params.pnp_method_enum = read_pnp_method(pnp_method);
  nh.getParam("downsample_point_cloud", slam_params.downsample_point_cloud);
  nh.getParam("voxel_size", slam_params.voxel_size);
  nh.getParam("nearby_vertex_check_num", slam_params.nearby_vertex_check_num);

  nh.getParam("use_ransac_for_pnp", slam_params.use_ransac_for_pnp);
  nh.getParam("do_ba_backend", slam_params.do_ba_backend);
  nh.getParam("min_depth", slam_params.min_depth);
  nh.getParam("max_depth", slam_params.max_depth);
  nh.getParam("verbose", slam_params.verbose);
  nh.getParam("pause_after_optimization", slam_params.pause_after_optimization);
  nh.getParam("initial_image_skip_num", slam_params.initial_image_skip_num);
  return slam_params;
}

void add_edges_to_nearby_vertices(const KeyFrameData &current_keyframe,
                                  const std::vector<KeyFrameData> &keyframes,
                                  const HandyCameraInfo &cam_info,
                                  const SLAMParams &slam_params) {
  int start_index = std::max(static_cast<int>(keyframes.size()) -
                                 slam_params.nearby_vertex_check_num,
                             0);
  int end_index =
      keyframes.size() -
      1; // keyframes.size()-1 because we already checked the last one
  // TODO
  std::cout << "Checking nearby vertices [from, to): " << start_index << ", "
            << end_index << std::endl;
  for (int i = start_index; i < end_index; ++i) {
    auto frame1_to_frame2 =
        front_end(cam_info, slam_params, keyframes.back(), current_keyframe);
    if (frame1_to_frame2.has_value()) {
      add_to_graph(current_keyframe, keyframes.back(), frame1_to_frame2.value(),
                   true);
    }
  }
}

int main(int argc, char *argv[]) {
  // Step 1: set up ros node, and parameters
  ros::init(argc, argv, "rgbd_rico_slam_mini");
  ros::NodeHandle nh;
  auto slam_params = read_params(nh);

  // Step 2: bagparser, topics and camera info
  ros::Publisher poses_pub =
      nh.advertise<geometry_msgs::PoseArray>("optimized_poses", 1, true);
  ros::Publisher points_pub =
      nh.advertise<PointCloud>("optimized_points", 1, true);
  SimpleRoboticsRosUtils::BagParser bp(nh, PACKAGE_NAME,
                                       "/data/rgbd_dataset_freiburg1_xyz.bag");
  bp.fast_forward(RGB_TOPIC, slam_params.initial_image_skip_num);
  bp.fast_forward(DEPTH_TOPIC, slam_params.initial_image_skip_num);

  // Step 3:using namespace RgbdSlamRico;using namespace RgbdSlamRico;
  // initialize data structures for optimization
  HandyCameraInfo cam_info = load_camera_info(bp, CAMERA_INFO_TOPIC);
  PointCloud::Ptr point_cloud(new PointCloud);
  point_cloud->header.frame_id = "camera";
  point_cloud->height = point_cloud->width = 1;

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
    // Step 5: detect ORB features
    auto orb_features = get_valid_orb_features(slam_params, current_keyframe);
    if (!orb_features.has_value())
      continue;
    current_keyframe.orb_res = orb_features.value();
    // Step 6: Initialize
    if (keyframes.empty()) {
      initialize_global_optimizer(slam_params.verbose, current_keyframe);
      keyframes.emplace_back(std::move(current_keyframe));
      continue;
    }

    if (slam_params.test_with_optimization) {
      auto frame1_to_frame2 =
          front_end(cam_info, slam_params, keyframes.back(), current_keyframe);
      if (!frame1_to_frame2.has_value())
        continue;
      add_to_graph(current_keyframe, keyframes.back(), frame1_to_frame2.value(),
                   true);

      if (slam_params.do_ba_backend) {
        add_edges_to_nearby_vertices(current_keyframe, keyframes, cam_info,
                                     slam_params);
      }
      // This pose is not used by global optimizer. This is for visualization
      // only
      current_keyframe.pose = keyframes.back().pose * frame1_to_frame2.value();
    }

    add_point_cloud(current_keyframe.pose, current_keyframe.image,
                    current_keyframe.depth_image, cam_info, point_cloud,
                    slam_params);
    if (slam_params.pause_after_optimization) {
      const std::string msg =
          "Iteration " + std::to_string(i) + ", Please hit enter pause";
      SimpleRoboticsCppUtils::sleep_or_pause_on_keystroke(
          SimpleRoboticsCppUtils::to_color_msg(
              SimpleRoboticsCppUtils::ANSIStrings::RED, msg, true),
          1500);
    }
    // so current_keyframe is invalidated
    keyframes.emplace_back(std::move(current_keyframe));
    visualize_slam_results(keyframes, poses_pub, points_pub, point_cloud);
  }

  // Step 10: global BA optimization (backend)
  ros::spin();

  return 0;
}
