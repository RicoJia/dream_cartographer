#!/usr/bin/env python3
"""
TODOs:
1. **The current scan matching method is subject to local minima, and is not feasible for large spaces**
1. Need scan message angular increments. Right now it's hardcoded
2. Map saved from map_server doesn't provide correct occupancy_threshold. We are hardcoding them
3. Need laser scan effective ranges
"""

import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle
import numpy as np
from typing import List, Tuple, Dict
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
    get_binary_image
)
import cv2
import os

SEARCH_GRID_RESOLUTION = 4  # 0.2m
BEAM_SEARCH_KERNEL_SIZE = 2
BEAM_NOICE_VARIANCE = 0.1  # in meter
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
    map_image = mpimg.imread(img_path)
    return map_metadata, map_image


def map_px_to_plot_coordinates(map_px, origin):
    return map_px - origin


def load_scan_messages(filename):
    with open(filename, "rb") as file:
        data = pickle.load(file)
    return data

def visualize_matrix(mat):
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
    map_image, origin_px, best_laser_endbeam_xy=None, p_free_endbeams=None, robot_pose=None
):
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

def rgba_to_grayscale(map_image):
    try:
        if map_image.shape[2] == 4:
            return cv2.cvtColor(map_image, cv2.COLOR_BGRA2GRAY)
        else:
            return map_image
    except Exception:
        return map_image

def template_matching_cuda(img, template):
    # Create CUDA objects
    img_gpu = cv2.cuda_GpuMat()  # Create GpuMat object
    template_gpu = cv2.cuda_GpuMat()

    # Upload data to the device
    img_gpu.upload(img)
    template_gpu.upload(template)
    # Create result matrix size
    result_cols = img.shape[1] - template.shape[1] + 1
    result_rows = img.shape[0] - template.shape[0] + 1

    # Create GpuMat for result
    result_gpu = cv2.cuda_GpuMat(result_rows, result_cols, cv2.CV_32FC1)

    # Perform template matching
    matcher = cv2.cuda.createTemplateMatching(cv2.CV_8UC1, cv2.TM_CCOEFF_NORMED)
    matcher.match(img_gpu, template_gpu, result_gpu)

    # Download result back to host
    result = result_gpu.download()
    return result


if __name__ == "__main__":
    map_metadata, map_image = load_map_and_meta_data(get_file_path_in_the_same_folder("bird_world.pgm"),
                                                     get_file_path_in_the_same_folder("bird_world.yaml"))
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

    def generate_smallest_matrix(point_list, orig_in_matrix_coord: np.ndarray):
        matrices = []
        relative_robot_poses_in_mat = []
        for points in point_list:
            x_coords = np.asarray([points[0] for points in points],dtype=int)
            y_coords = np.asarray([points[1] for points in points], dtype=int)
            min_x = min(x_coords)
            min_y = min(y_coords)
            max_x = max(x_coords)
            max_y = max(y_coords)
            new_mat = np.ones((max_x - min_x + 1, max_y - min_y + 1))
            x_coords -= min_x
            y_coords -= min_y 
            for x,y in zip(x_coords,y_coords):
                new_mat[x][y] = 0
            matrices.append(new_mat)
            relative_robot_poses_in_mat.append(
                orig_in_matrix_coord[0][0] - np.array([min_x, min_y])
            )
        return matrices, relative_robot_poses_in_mat

    def score(p_hits_matrix):
        p_hits = p_hits_matrix
        try:
            xor_mask = create_mask(p_hits, img_width, img_height)
        except IndexError:
            return -1
        # TODO if works, can further
        # result_map = (map_image == xor_mask).astype(int)
        gradient_laser_scan = get_gradient_mat(xor_mask)
        result_map = (gradient_laser_scan == img_gradient).astype(int)
        return np.sum(result_map)

    def optimize_using_template_matching(
        map_image: np.ndarray,
        top_left: np.ndarray,
        bottom_right: np.ndarray,
        search_grid_resolution: int,
        best_point_so_far: np.ndarray,
    ):
        start_time = time.time()
        p_hits_for_all_thetas_for_pose_matrix = map_pixel_to_matrix(
            p_hits_for_all_thetas, origin_px, img_height
        )
        origin_in_matrix_coords = map_pixel_to_matrix(
            [np.array([[0,0], ])], origin_px, img_height
        )
        p_hits_for_all_thetas_images, relative_robot_poses_in_mat = generate_smallest_matrix(p_hits_for_all_thetas_for_pose_matrix, origin_in_matrix_coords)
        point_candidates = []
        theta_id_candidates = []
        p_hits_relative_matrix = []

        binary_map_image = get_binary_image(map_image)
        for theta_id, p_hits_for_all_thetas_image in enumerate(p_hits_for_all_thetas_images):
            #TODO Remember to remove
            template = p_hits_for_all_thetas_image.astype(np.uint8)
            # TODO
            res = cv2.matchTemplate(binary_map_image, template, cv2.TM_CCOEFF_NORMED)
            # res = template_matching_cuda(binary_map_image, template)
            candidate_mat_coords = np.where(res > TEMPLATE_MATCHING_THRESHOLD)
            #TODO Remember to remove
            # print(f'Rico: candidate_mat_coords: {candidate_mat_coords}, theta: {search_thetas[theta_id]}')
            for x,y in zip(candidate_mat_coords[0], candidate_mat_coords[1]):
                max_loc = np.array([x,y])
                max_loc += relative_robot_poses_in_mat[theta_id]
                point_candidates.append(max_loc)
                theta_id_candidates.append(theta_id)
                p_hits_relative_matrix.append(p_hits_for_all_thetas_images[theta_id])

        best_score = -1
        best_point_so_far = None
        best_p_hits = None
        for point, theta_id, p_hits_relative_mat_single in zip(point_candidates, theta_id_candidates, p_hits_relative_matrix): 
            # To matrix coords
            max_loc_map_pose = matrix_to_map_pixel(np.array([point]), origin_px, img_height)
            theta = search_thetas[theta_id] 
        #     # visualize_map(p_hits_for_all_thetas_image, origin_px)
            p_hits_for_theta_absolute = add_pose_to_relative_poses(
                [p_hits_for_all_thetas[theta_id]], max_loc_map_pose
            )
            p_hits_relative_mat_single = map_pixel_to_matrix(
                p_hits_for_theta_absolute, origin_px, img_height
            )[0]
            s = score(p_hits_relative_mat_single)
            # print(f'Rico: score: {s}, theta: {point}')
            if s > best_score:
                best_score = s
                best_point_so_far = point
                best_p_hits = p_hits_for_theta_absolute[0]
        #TODO Remember to remove
        print(f'Rico: best_point_so_far: {best_point_so_far}, score: {best_score}')
        print(f'Rico: total_time: {time.time() - start_time}')
        #TODO Remember to remove
        print(f'Rico: {best_point_so_far}')
        best_point_so_far = matrix_to_map_pixel(np.array([best_point_so_far, ]), origin_px, img_height)
        visualize_map(map_image, origin_px, best_p_hits, robot_pose=best_point_so_far[0])
            
            
    # optimize_using_grid_search(
    #     map_image=map_image,
    #     top_left=np.array([0, 0]),
    #     bottom_right=np.asarray(map_image.shape),
    #     search_grid_resolution=SEARCH_GRID_RESOLUTION,
    #     best_point_so_far=None,
    # )
    optimize_using_template_matching(
        map_image=map_image,
        top_left=np.array([0, 0]),
        bottom_right=np.asarray(map_image.shape),
        search_grid_resolution=SEARCH_GRID_RESOLUTION,
        best_point_so_far=None, 
    )
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
