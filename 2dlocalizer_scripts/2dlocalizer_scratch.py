def score_pose(map, laser_endbeam_xy, pose):
  #TODO - this can be delegated to CUDA
  score = 0.0
  # TODO
  debug_match_num = 0
  for endpoint in laser_endbeam_xy:
    p_free_offset = _get_vector_1_pixel_away(endpoint, pose[:2])
    best_mu_squared = 400
    match_found = False
    endpoint_x, endpoint_y = endpoint
    for xx in beam_search_kernel:
      for yy in beam_search_kernel:
        p_hit = np.array([endpoint_x + xx, endpoint_y + yy])
        # TODO?
        # if np.all(image_lower_bound <  and p_hit < image_upper_bound):
        #   continue
        p_free = p_hit + p_free_offset
        if (map[p_hit[0], p_hit[1]] == MapValue.OCC.value) and (map[p_free[0], p_free[1]] == MapValue.FREE.value):
          mu_squared = xx*xx + yy * yy
          if mu_squared < best_mu_squared:
            best_mu_squared = mu_squared
          match_found = True
    if match_found:
      score += np.exp(-best_mu_squared * resolution_squared/BEAM_NOICE_VARIANCE)
      debug_match_num +=1
  print("debug match num: ", debug_match_num)
  return score

def optimize_pose(map, laser_scan, pose):
  #pose is pixel value
  angle_step = np.pi/8.0
  # TODO
  NUM_ITERATIONS = 1
  linear_step = 2**NUM_ITERATIONS
  motions = [np.array([0,0,0.0])]
  # TODO
  for i in range(NUM_ITERATIONS):
    angle_step /= 2.0
    linear_step = int(linear_step/2)
    # TODO - important
    motions += [
      # np.array([linear_step,0,0]),
      # np.array([0,linear_step,0]),
      # np.array([-linear_step,0,0]),
      # np.array([0,-linear_step,0]),
      # np.array([0,0,angle_step]),
      # np.array([0,0,-angle_step]),
    ]

  best_score = -1
  best_pose = None
  best_laser_endbeam_xy = None
  for motion in motions:
    new_pose = pose + motion
    laser_endbeam_xy = transform_laser_scan(laser_scan=laser_scan, pose=new_pose)
    local_score = score_pose(map=map, laser_endbeam_xy =laser_endbeam_xy , pose=new_pose)
  #   # print(laser_endbeam_xy)
    print(new_pose, local_score)
    if local_score > best_score:
      best_score = local_score
      best_pose = new_pose
      best_laser_endbeam_xy = laser_endbeam_xy
  return best_score, best_pose, best_laser_endbeam_xy

###########################################################
### Threadpool
###########################################################

from concurrent.futures import ProcessPoolExecutor
import numpy as np
import cv2

def process_image(theta_id_and_image):
    theta_id, p_hits_for_all_thetas_image = theta_id_and_image
    template = p_hits_for_all_thetas_image.astype(np.uint8)
    res = cv2.matchTemplate(binary_map_image, template, cv2.TM_CCOEFF_NORMED)
    candidate_mat_coords = np.where(res > TEMPLATE_MATCHING_THRESHOLD)
    
    results = []
    for x, y in zip(candidate_mat_coords[0], candidate_mat_coords[1]):
        max_loc = np.array([x, y])
        max_loc += relative_robot_poses_in_mat[theta_id]
        results.append((max_loc, theta_id, p_hits_for_all_thetas_image))
    return results

# Assuming 'p_hits_for_all_thetas_images' and 'relative_robot_poses_in_mat' are defined
# Prepare data for processing
theta_id_and_images = list(enumerate(p_hits_for_all_thetas_images))

# Use a ProcessPoolExecutor to run things in parallel
with ProcessPoolExecutor() as executor:
    results = executor.map(process_image, theta_id_and_images)

# Flatten list of lists and extract results
point_candidates = []
theta_id_candidates = []
p_hits_relative_matrix = []

for result in results:
    for max_loc, theta_id, image in result:
        point_candidates.append(max_loc)
        theta_id_candidates.append(theta_id)
        p_hits_relative_matrix.append(image)

        
    # optimization
    # 1. Search grid
    def optimize_using_grid_search(
        map_image: np.ndarray,
        top_left: np.ndarray,
        bottom_right: np.ndarray,
        search_grid_resolution: int,
        best_point_so_far: np.ndarray,
    ):
        if search_grid_resolution == 0:
            return
        search_grid_points = get_points_on_search_grid(
            map_image=map_image,
            top_left=top_left,
            bottom_right=bottom_right,
            search_grid_resolution=search_grid_resolution,
        )
        search_grid_points_map_pixelized = matrix_to_map_pixel(
            search_grid_points, origin_px, img_height=img_height
        )
        # TODO
        # search_grid_points_map_pixelized =  np.array([[0,0]])
        best_score = 0
        best_point = None
        best_theta_index = -1
        best_p_hits = None
        for pose in search_grid_points_map_pixelized:
            # all map coords thetas
            p_hits_for_all_thetas_for_pose = add_pose_to_relative_poses(
                p_hits_for_all_thetas, pose[:2]
            )
            p_frees_for_all_thetas_for_pose = add_pose_to_relative_poses(
                p_frees_for_all_thetas, pose[:2]
            )
            # To matrix coords
            p_hit_in_matrix_coords_for_all_thetas = map_pixel_to_matrix(
                points_for_all_thetas=p_hits_for_all_thetas_for_pose,
                origin_px=origin_px,
                img_height=img_height,
            )
            p_free_in_matrix_coords_for_all_thetas = map_pixel_to_matrix(
                points_for_all_thetas=p_frees_for_all_thetas_for_pose,
                origin_px=origin_px,
                img_height=img_height,
            )
            best_single_pose_score = 0
            best_single_pose_theta_index = -1
            best_single_pose_p_hits = None
            for theta_idx in range(len(search_thetas)):
                p_hits = p_hit_in_matrix_coords_for_all_thetas[theta_idx]
                p_frees = p_free_in_matrix_coords_for_all_thetas[theta_idx]
                try:
                    xor_mask = create_mask(p_hits, img_width, img_height)
                except IndexError:
                    continue
                # TODO if works, can further
                # result_map = (map_image == xor_mask).astype(int)
                gradient_laser_scan = get_gradient_mat(xor_mask)
                result_map = (gradient_laser_scan == img_gradient).astype(int)
                score = np.sum(result_map)

                # #TODO Remember to remove
                # print(f'Rico: score: {score}')
                # print(f'Rico: image gradient')
                # visualize_map(img_gradient, origin_px)
                # print(f'Rico: laser scan gradient')
                # visualize_map(gradient_laser_scan, origin_px)
                # print(f'Rico: result map')
                # visualize_map(result_map, origin_px)
                if score > best_single_pose_score:
                    best_single_pose_score = score
                    best_single_pose_theta_index = theta_idx
                    best_single_pose_p_hits = p_hits_for_all_thetas_for_pose[theta_idx]
                # print(f'Theta: {search_thetas[theta_idx]} Score: {score}')
            if best_score < best_single_pose_score:
                best_score = best_single_pose_score
                best_point = pose
                best_theta_index = best_single_pose_theta_index
                best_p_hits = best_single_pose_p_hits
            # print(
            #     f"best angle {best_single_pose_theta_index}, score: {best_single_pose_score}"
            # )
        print(
            f"best score: {best_score}, best point: {best_point}, best theta: {search_thetas[best_theta_index]}"
        )
        quarter_window = np.array([search_grid_resolution, search_grid_resolution])
        visualize_map(map_image, origin_px, best_laser_endbeam_xy=best_p_hits)
        # TODO
        # if best_point_so_far is not None and best_point is not None:
        #     if np.array_equal(best_point, best_point_so_far):
        #         visualize_map(map_image, origin_px, best_laser_endbeam_xy=best_p_hits)
        # return
        optimize_using_grid_search(
            map_image=map_image,
            top_left=map_pixel_to_matrix(
                [np.array([best_point])], origin_px, img_height
            )[0][0]
            - quarter_window,
            bottom_right=map_pixel_to_matrix(
                [np.array([best_point])], origin_px, img_height
            )[0][0]
            + quarter_window,
            search_grid_resolution=int(search_grid_resolution / 2),
            best_point_so_far=best_point,
        )
    # optimize_using_grid_search(
    #     map_image=map_image,
    #     top_left=np.array([0, 0]),
    #     bottom_right=np.asarray(map_image.shape),
    #     search_grid_resolution=SEARCH_GRID_RESOLUTION,
    #     best_point_so_far=None,
    # )

    # Score:
    # Given a pose in search grid,
    #   - map_pixel of laser endpoint beam relative to the pose?
    #   - map_pixel of p_free relative to the pose (could be optimized potentially)
    # - Transform:
    #   - Indices of the laser endpoint beams in the matrix
    #   - Indices of the p_free in the matrix
    # - Masking:
    #   - Create mask with [0,1] of p_hit and p_free
    #   - score = sum(map xor mask)