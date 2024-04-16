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