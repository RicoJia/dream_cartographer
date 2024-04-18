#!/usr/bin/env python3
"""
TODOs:
1. **The current scan matching method is subject to local minima, and is not feasible for large spaces**
1. Need scan message angular increments. Right now it's hardcoded
2. Map saved from map_server doesn't provide correct occupancy_threshold. We are hardcoding them
3. Need laser scan effective ranges
"""

import yaml

# import matplotlib.image as mpimg
import pickle
import numpy as np
from typing import List, Tuple, Dict, Deque
import time
from localizer_2d_utils import (
    get_vector_1_pixel_away_vectorized,
    map_pixel_to_matrix,
    create_mask,
    MapValue,
    matrix_to_map_pixel,
    add_pose_to_relative_poses,
    get_points_on_search_grid,
    get_gradient_mat,
    get_binary_image,
)
import cv2
import os
from multiprocessing import Pool
from collections import deque

SEARCH_GRID_RESOLUTION = 4  # 0.2m
BEAM_SEARCH_KERNEL_SIZE = 2
BEAM_NOICE_VARIANCE = 0.1  # in meter
# TODO
# NUM_ANGULAR_SEG = 128
NUM_ANGULAR_SEG = 128
TEMPLATE_MATCHING_THRESHOLD = 0.25


def get_file_path_in_the_same_folder(filename):
    # To get the full, absolute file path
    absolute_file_path = os.path.abspath(__file__)
    dir_name = os.path.dirname(absolute_file_path)
    return os.path.join(dir_name, filename)


def load_map_and_meta_data(img_path, metadata_path):
    # Replace 'map.yaml' with the path to your uploaded YAML file
    with open(metadata_path, "r") as file:
        map_metadata = yaml.safe_load(file)
    # Replace 'map.pgm' with the path to your uploaded PGM file
    # map_image = mpimg.imread(img_path)
    map_image = cv2.imread(img_path)
    return map_metadata, map_image


def map_px_to_plot_coordinates(map_px, origin):
    return map_px - origin


def load_scan_messages(filename):
    with open(filename, "rb") as file:
        data = pickle.load(file)
    return data


def visualize_matrix(mat):
    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 10))
    plt.imshow(
        mat,
        cmap="gray",
    )  # Maps are often best visualized in grayscale

    plt.title("Matrix Visualization")
    plt.xlabel("Matrix X")
    plt.ylabel("Matrix Y")
    plt.show()


def visualize_map(
    map_image,
    origin_px,
    best_laser_endbeam_xy=None,
    p_free_endbeams=None,
    robot_pose=None,
):
    import matplotlib.pyplot as plt

    origin_x_px, origin_y_px = origin_px
    plt.figure(figsize=(10, 10))
    img_width = map_image.shape[1]
    img_height = map_image.shape[0]
    x_ticks = np.arange(-origin_x_px, img_width - origin_x_px)
    y_ticks = np.arange(-origin_y_px, img_height - origin_y_px)
    plt.imshow(
        map_image,
        cmap="gray",
        extent=[x_ticks[0], x_ticks[-1], y_ticks[0], y_ticks[-1]],
    )  # Maps are often best visualized in grayscale

    plt.title("Map Visualization")
    plt.xlabel("Matrix Y(plot X)")
    plt.ylabel("Matrix X(plot Y)")
    map_origin_px = np.array([0, 0, 0])
    plt.plot(map_origin_px[0], map_origin_px[1], "ro")  # Red dot
    plt.text(map_origin_px[0], map_origin_px[1], "Origin", color="red")
    if robot_pose is not None:
        plt.plot(robot_pose[0], robot_pose[1], "ro")  # Red dot
        plt.text(robot_pose[0], robot_pose[1], "robot", color="purple")

    # One Big Difference between matplotlib plot and np array is x and y in np array row major.
    # The visualization here is column major.
    def show_scatter_plot(points: List[np.ndarray], color: str, label: str):
        x_coords = [xy[0] for xy in points]
        y_coords = [xy[1] for xy in points]

        plt.scatter(
            x_coords, y_coords, c=color, marker="o", label=label, s=2, linewidths=0.05
        )
        # Adding a legend to identify the points
        plt.legend()

    if best_laser_endbeam_xy is not None:
        show_scatter_plot(
            points=best_laser_endbeam_xy, color="red", label="Laser Endpoints"
        )

    if p_free_endbeams is not None:
        show_scatter_plot(
            points=p_free_endbeams, color="green", label="Last Free Points"
        )

    plt.show()


def transform_laser_scan(laser_scan, pose):
    # Get laser scan (x,y) in world frame
    # Convert (x, y) to homogeneous coordinates
    angles = np.arange(0, -2 * np.pi, -2 * np.pi / len(laser_scan))
    x, y, theta = pose
    laser_beam_endpoints = [
        np.array(
            [
                int(x + range / resolution * np.cos(theta + bearing)),
                int(y + range / resolution * np.sin(theta + bearing)),
            ]
        )
        for range, bearing in zip(laser_scan, angles)
        if range < effective_range
    ]
    return laser_beam_endpoints


def get_laser_endbeam_relative_for_all_thetas(
    search_thetas: np.ndarray,
    bearings: np.ndarray,
    scan_msg: np.ndarray,
    effective_range: float,
    resolution: float,
):
    # return array doesn't have inf. And they are in pixelized map frame
    assert len(scan_msg) == len(
        bearings
    ), "number of bearings and scan_msg should be the same"
    p_hits_for_all_thetas = []
    # [theta1[(x,y)...], theta2 [...]]
    for theta in search_thetas:
        p_hit_xs_relative = [
            np.round(d * np.cos(bearing + theta) / resolution)
            for d, bearing in zip(scan_msg, bearings)
            if d <= effective_range
        ]  # x = d*cos(theta+bearing)
        p_hit_ys_relative = [
            np.round(d * np.sin(bearing + theta) / resolution)
            for d, bearing in zip(scan_msg, bearings)
            if d <= effective_range
        ]  # y = d*sin(theta+bearing)
        p_hits_for_all_thetas.append(
            np.asarray(
                [
                    np.array((x, y), dtype=int)
                    for x, y in zip(p_hit_xs_relative, p_hit_ys_relative)
                ]
            )
        )
    # combine_xy_for_all_thetas
    return p_hits_for_all_thetas


def get_p_frees_for_all_thetas(
    search_thetas: np.ndarray, p_hits_for_all_thetas: List[np.ndarray]
):
    # [theta1[np.array(x,y)...], theta2 [...]]
    p_free_relative = []
    for p_hits_for_theta in p_hits_for_all_thetas:
        unit_vecs = get_vector_1_pixel_away_vectorized(
            p_hits_for_theta, np.array([0, 0])
        )
        p_frees_for_theta = p_hits_for_theta + unit_vecs
        p_free_relative.append(p_frees_for_theta)

    return p_free_relative


def downsample(image, factor):
    new_width = image.shape[1] // factor
    new_height = image.shape[0] // factor

    # Resize the image
    downsampled_image = cv2.resize(
        image, (new_width, new_height), interpolation=cv2.INTER_NEAREST
    )
    return downsampled_image


def rgba_to_grayscale(map_image):
    try:
        if map_image.shape[2] == 4:
            return cv2.cvtColor(map_image, cv2.COLOR_BGRA2GRAY)
        elif map_image.shape[2] == 3:
            return cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
        else:
            return map_image
    except Exception:
        return map_image


def generate_smallest_matrix(point_list, orig_in_matrix_coord: np.ndarray):
    matrices = []
    relative_robot_poses_in_mat = []
    for points in point_list:
        x_coords = np.asarray([points[0] for points in points], dtype=int)
        y_coords = np.asarray([points[1] for points in points], dtype=int)
        min_x = min(x_coords)
        min_y = min(y_coords)
        max_x = max(x_coords)
        max_y = max(y_coords)
        new_mat = np.ones((max_x - min_x + 1, max_y - min_y + 1))
        x_coords -= min_x
        y_coords -= min_y
        for x, y in zip(x_coords, y_coords):
            new_mat[x][y] = 0
        matrices.append(new_mat)
        relative_robot_poses_in_mat.append(
            orig_in_matrix_coord[0][0] - np.array([min_x, min_y])
        )
    return matrices, relative_robot_poses_in_mat


def vanilla_score(p_hits_single):
    try:
        xor_mask = create_mask(p_hits_single, img_width, img_height)
    except IndexError:
        return -1
    # gradient_laser_scan = get_gradient_mat(xor_mask)
    result_map = (xor_mask == map_image).astype(int)
    return np.sum(result_map)


def score(p_hits):
    try:
        xor_mask = create_mask(p_hits, img_width, img_height)
    except IndexError:
        return -1
    gradient_laser_scan = get_gradient_mat(xor_mask)
    result_map = (gradient_laser_scan == img_gradient).astype(int)
    return np.sum(result_map)


def find_score(args) -> Tuple[Deque, Deque, Deque]:
    """Find score for a single theta.

    Args:
        args (_type_): _description_
    """
    (
        binary_map_image,
        theta_id,
        p_hits_for_all_thetas_image_single_theta,
        relative_robot_poses_in_mat_theta,
        origin_px,
        img_height,
    ) = args
    template = p_hits_for_all_thetas_image_single_theta.astype(np.uint8)
    # binary_map_image = downsample(binary_map_image, 4)
    # template = downsample(template, 4)
    res = cv2.matchTemplate(binary_map_image, template, cv2.TM_CCOEFF_NORMED)
    candidate_mat_coords = np.where(res > TEMPLATE_MATCHING_THRESHOLD)
    # p_hits_relative_matrix = []
    # theta = search_thetas[theta_id]
    # Got a list of candidates. try to score them.
    scores = deque()
    points = deque()
    p_hits = deque()
    # TODO
    for x, y in zip(candidate_mat_coords[0], candidate_mat_coords[1]):
        max_loc = np.array([x, y])
        max_loc += relative_robot_poses_in_mat_theta
        # point_candidates.append(max_loc)
        # p_hits_relative_matrix.append(p_hits_for_all_thetas_images[theta_id])
        # To matrix coords
        max_loc_map_pose = matrix_to_map_pixel(
            np.array([max_loc]), origin_px, img_height
        )
        #     # visualize_map(p_hits_for_all_thetas_image, origin_px)
        p_hits_for_theta_absolute = add_pose_to_relative_poses(
            [p_hits_for_all_thetas[theta_id]], max_loc_map_pose
        )
        p_hits_relative_mat_single = map_pixel_to_matrix(
            p_hits_for_theta_absolute, origin_px, img_height
        )[0]
        scores.append(
            # score(p_hits_relative_mat_single)
            vanilla_score(p_hits_relative_mat_single)
        )
        points.append(max_loc)
        p_hits.append(p_hits_for_theta_absolute)
    return scores, points, p_hits


def optimize_using_template_matching(
    map_image: np.ndarray,
):
    start_time = time.time()
    p_hits_for_all_thetas_for_pose_matrix = map_pixel_to_matrix(
        p_hits_for_all_thetas, origin_px, img_height
    )
    origin_in_matrix_coords = map_pixel_to_matrix(
        [
            np.array(
                [
                    [0, 0],
                ]
            )
        ],
        origin_px,
        img_height,
    )
    (
        p_hits_for_all_thetas_images,
        relative_robot_poses_in_mat,
    ) = generate_smallest_matrix(
        p_hits_for_all_thetas_for_pose_matrix, origin_in_matrix_coords
    )
    binary_map_image = get_binary_image(map_image)

    pool = Pool()
    THETA_NUM = len(search_thetas)
    task_args = zip(
        [binary_map_image] * THETA_NUM,
        list(range(THETA_NUM)),
        p_hits_for_all_thetas_images,
        relative_robot_poses_in_mat,
        [origin_px] * THETA_NUM,
        [img_height] * THETA_NUM,
    )
    results = pool.map(find_score, task_args)
    pool.close()
    pool.join()

    # Find the result with the best score
    best_score = -np.inf
    best_point_so_far = None
    best_p_hits = None

    for scores, points, p_hits_ls in results:
        for s, point, p_hits in zip(scores, points, p_hits_ls):
            # TODO Remember to remove
            print(f"Rico: s: {s}, point: {point}")
            if s > best_score:
                best_score = s
                best_point_so_far = point
                # TODO this is ugly
                best_p_hits = p_hits[0]

    print(f"Rico: best_point_so_far: {best_point_so_far}, score: {best_score}")
    print(f"Rico: total_time: {time.time() - start_time}")

    if best_point_so_far is not None:
        best_point_so_far = matrix_to_map_pixel(
            np.array(
                [
                    best_point_so_far,
                ]
            ),
            origin_px,
            img_height,
        )
        visualize_map(
            map_image, origin_px, best_p_hits, robot_pose=best_point_so_far[0]
        )


if __name__ == "__main__":
    map_metadata, map_image = load_map_and_meta_data(
        get_file_path_in_the_same_folder("map499.png"),
        get_file_path_in_the_same_folder("map499.yaml"),
    )
    # map_metadata, map_image = load_map_and_meta_data(get_file_path_in_the_same_folder("bird_world.pgm"),
    #                                                  get_file_path_in_the_same_folder("bird_world.yaml"))
    map_image = rgba_to_grayscale(map_image)
    all_data = load_scan_messages(get_file_path_in_the_same_folder("scan_data.pkl"))
    img_gradient = get_gradient_mat(map_image)
    resolution = map_metadata["resolution"]
    img_width = map_image.shape[1]
    img_height = map_image.shape[0]
    # [10/resolution, 10/resolution] in m. This is the pixel coord of the map origin,
    # relative to the bottom left corner of the image
    origin_px = (
        np.array([-map_metadata["origin"][0], -map_metadata["origin"][1]]) / resolution
    ).astype(int)

    # # Convert world coordinates to pixel coordinates
    # # Especially y, because in robotics, Y starts from bottoms up
    # # while in computer graphics, Y starts from top down
    # visualize_map(map_image, origin_px)
    beam_search_kernel = list(
        range(-BEAM_SEARCH_KERNEL_SIZE, BEAM_SEARCH_KERNEL_SIZE, 1)
    )
    # meter -> pixels
    effective_range = 10 / resolution
    resolution_squared = resolution * resolution
    # TODO Remember to remove
    print(f"length of data: {len(all_data)}")
    trial_scan_msg = all_data[-1]

    # TODO
    search_thetas = np.arange(0, 2 * np.pi, np.pi / NUM_ANGULAR_SEG)
    # search_thetas = [np.pi/2]
    bearings = np.arange(0, 2 * np.pi, 2 * np.pi / len(trial_scan_msg))
    # From now on, we are operating in pixelized map frame
    # [theta1[(x,y)...], theta2 [...]]
    p_hits_for_all_thetas = get_laser_endbeam_relative_for_all_thetas(
        search_thetas=search_thetas,
        bearings=bearings,
        scan_msg=trial_scan_msg,
        effective_range=effective_range,
        resolution=resolution,
    )
    p_frees_for_all_thetas = get_p_frees_for_all_thetas(
        search_thetas, p_hits_for_all_thetas
    )

    optimize_using_template_matching(
        map_image=map_image,
    )
