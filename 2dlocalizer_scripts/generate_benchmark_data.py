#!/usr/bin/env python3
"""
1. Take in an image (D)
2. Try to load pre-generated grid points, based on map name and grid step multiplier
3. If no pre-generated grid points, get a group of grid points on the image, if the point is free (D)
    1. The orientation of the robot is randomly generated
4. For a given max_range, 
    2. Sythesize ray tracing data. If there are points outside of the range, inf will be returned
5. Save the range data, and the pose into two csv file. range_data| 
"""

import argparse
import csv
import os
from collections import deque
from typing import List

import numpy as np
from localizer_2d_utils import (MapValue, get_file_path_in_the_same_folder,
                                img_pixel_to_map, load_map_and_meta_data,
                                map_to_img_pixel, rgba_to_grayscale)
import matplotlib.pyplot as plt

# Must be integer, this is relative to the map resolution
GRID_STEP_MULTIPLIER = 300
SAVED_DATA_FOLDER = "data"
NUM_ANGLES = 360
# in meters
MAX_RANGE = 6


def get_paths():
    parser = argparse.ArgumentParser()
    # TODO
    parser.add_argument("--img_name", type=str, default="map499.png")
    # parser.add_argument("--img_name", type=str, required=True, default="map499.png")
    args = parser.parse_args()
    img_path = os.path.join(SAVED_DATA_FOLDER, args.img_name)
    prepended_map_name = ".".join(img_path.split(".")[:-1])
    metadata_path = prepended_map_name + ".yaml"
    BENCHMARK_RANGE_DATA_FILE = (
        f"{prepended_map_name}_range_data_{GRID_STEP_MULTIPLIER}.csv"
    )
    BENCHMARK_POSES_FILE = (
        f"{prepended_map_name}_poses_{GRID_STEP_MULTIPLIER}.csv"
    )
    img_path = get_file_path_in_the_same_folder(img_path)
    metadata_path = get_file_path_in_the_same_folder(metadata_path)
    benchmark_poses_path = get_file_path_in_the_same_folder(
        BENCHMARK_POSES_FILE
    )
    benchmark_range_data_path = get_file_path_in_the_same_folder(
        BENCHMARK_RANGE_DATA_FILE
    )
    return (
        img_path,
        metadata_path,
        benchmark_poses_path,
        benchmark_range_data_path,
    )

def get_free_map_img(map_image, map_metadata):
    height, width = map_image.shape
    free_thresh, negate = map_metadata["free_thresh"], bool(
        map_metadata["negate"]
    )
    """Return a boolean map of free cells (free = True)"""
    # This section comes straight from http://wiki.ros.org/map_server
    if negate:
        # val < int_free_thresh are free val > int_occupied_thresh are occupied
        int_free_thresh = int(255 * free_thresh)
        free_map_img = map_image < int_free_thresh
        print(f"Not negating the map")
    else:
        # val > int_free_thresh are free val < int_occupied_thresh are occupied
        int_free_thresh = 255 - int(255 * free_thresh)
        free_map_img = map_image > int_free_thresh
        # TODO Remember to remove
        print(f"Negating the map")
    return free_map_img

def generate_grid_poses(
    map_metadata, free_map_img, origin_px
) -> List[np.ndarray]:
    """Return the map pixel poses on a grid that are free pixels"""
    # the grid is like a checker board

    height, width = free_map_img.shape
    map_poses = list()
    for hy in range(GRID_STEP_MULTIPLIER, height, GRID_STEP_MULTIPLIER):
        for hx in range(GRID_STEP_MULTIPLIER, width, GRID_STEP_MULTIPLIER):
            if free_map_img[hy, hx] == True:
                map_pixel_point = img_pixel_to_map(
                    image_coords=np.array((hx, hy)),
                    img_height=height,
                    origin_px=origin_px,
                )
                # TODO
                # theta = np.random.uniform(0, 2 * np.pi)
                theta = 0.0
                map_poses.append(np.append(map_pixel_point, [theta]))
    return map_poses


def save_grid_poses(map_poses, benchmark_poses_path):
    with open(benchmark_poses_path, "w") as poses_file:
        writer = csv.writer(poses_file)
        writer.writerow(["x", "y", "theta"])
        for row in map_poses:
            writer.writerow(row)
    print(f"Rico: saved to {benchmark_poses_path}")


# return a deque of poses that do not have generated range data yet
def find_existing_grid_poses(
    benchmark_poses_path, benchmark_range_data_path
) -> List[np.ndarray]:
    poses = []
    if not os.path.exists(benchmark_poses_path):
        return poses
    with open(benchmark_poses_path, "r") as poses_file:
        reader = csv.DictReader(poses_file)
        for row in reader:
            poses.append(np.array([int(float(row["x"])), int(float(row["y"])), float(row["theta"])]))

    row_count = 0
    try:
        with open(benchmark_range_data_path, "r") as range_data_file:
            reader = csv.DictReader(poses_file)
            for _ in reader:
                row_count += 1
    except OSError:
        pass
    return_list = poses[row_count:]
    print(f"Found {len(return_list)} unfinished pre-generated poses")
    return return_list


def bresenham_line(start: np.ndarray, end: np.ndarray) -> List:
    delta = np.abs(end - start)
    step = delta / (end - start)
    dx, dy = delta[0], delta[1]
    sx, sy = step[0], step[1]
    x, y = start[0], start[1]
    return_pixels = []
    if dx > dy:
        err = dy / 2
        while x != end[0]:
            return_pixels.append(np.array([x, y], dtype=int))
            err += dy
            if err > dx:
                y += sy
                err -= sx
            x += sx
    else:
        err = dx /2
        while y != end[1]:
            return_pixels.append(np.array([x,y], dtype=int))
            err += dx
            if err > dy:
                x += sx
                err -= dy
            y += sy
    return_pixels.append(end)    
    return return_pixels
            
def test_bresenham_line():
    start = np.array([0,0])
    end = np.array([-3,-4])
    pixels = bresenham_line(start=start, end=end)
    x_vals, y_vals = zip(*pixels)

    # What does this do?
    plt.figure(figsize=(6,6))
    plt.plot(x_vals, y_vals, 'ro-')
    plt.grid(True)
    plt.show()


def synthesize_ray_tracing_data(
    max_range_pixel: np.ndarray,
    map_poses: List[np.ndarray],
    free_map_img: np.ndarray,
    benchmark_range_data_path: str,
    origin_px: np.ndarray,
):
    height, width = free_map_img.shape
    angles = np.arange(0, 2 * np.pi, 2 * np.pi/NUM_ANGLES)
    # [x, y, theta], theta is relative to the orthogonal map pose
    right_on_bounds = lambda a, upper_bound: a ==0 or a == upper_bound
    all_range_data = deque()
    for pose in map_poses:
        xy = pose[:2]
        theta = pose[2]
        img_pixel = map_to_img_pixel(map_point = xy, origin_px=origin_px, img_height=height).astype(int)
        range_data = deque()
        # TODO - to remove
        debug_occupied_cells = []
        for angle in angles:
            theta_map = theta + angle
            end_point = (img_pixel + max_range_pixel * np.array([np.cos(theta_map), np.sin(theta_map)])).astype(int)
            points = bresenham_line(start=img_pixel, end=end_point)
            max_range_hit = False
            for i in range(len(points)):
                p = points[i]
                # must swap y and x for img coords
                if right_on_bounds(p[0], height-1) or right_on_bounds(p[1], width-1) or \
                    not free_map_img[p[1], p[0]]:
                    map_distance = i/len(points) * MAX_RANGE
                    range_data.append(map_distance)
                    max_range_hit = True
                    # TODO 
                    map_endbeam = img_pixel_to_map(p, img_height=height, origin_px=origin_px)
                    debug_occupied_cells.append(map_endbeam)
                    break
            if not max_range_hit:
                range_data.append(MAX_RANGE)
                #TODO
                map_endbeam = img_pixel_to_map(end_point, img_height=height, origin_px=origin_px)
                debug_occupied_cells.append(map_endbeam)
            # #TODO Remember to remove
            print(f'==============================================================================')
            # print(f'start: img_pixel: {img_pixel} end: {end_point}')
            # #TODO Remember to remove
            # print(f'Range data: {range_data}')
            # #TODO Remember to remove
            print(f'Debug Occupied Cells: {debug_occupied_cells}')
        all_range_data.append(range_data)            
        # TODO
        debug_viz(debug_occupied_cells, free_binary_map, pose)
    return all_range_data

def debug_viz(debug_occupied_cells, map_image, robot_pose=None):
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
        plt.plot(robot_pose[0], robot_pose[1], "go")  # Green dot
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

    show_scatter_plot(
        points=debug_occupied_cells, color="red", label="Laser Endpoints"
    )


    plt.show()
    

def visualize_map_with_range_data(
        synthesized_range_data,
        unfinished_grid_poses,
        map_image
    ):
    pass

if __name__ == "__main__":
    (
        img_path,
        metadata_path,
        benchmark_poses_path,
        benchmark_range_data_path,
    ) = get_paths()
    #TODO Remember to remove
    print(f'Rico: metadata path: {img_path}')
    map_metadata, map_image = load_map_and_meta_data(
        img_path=img_path, metadata_path=metadata_path
    )
    map_image = rgba_to_grayscale(map_image)
    free_binary_map = get_free_map_img(map_image, map_metadata)
    resolution = map_metadata["resolution"]
    origin_px = (
        np.array([-map_metadata["origin"][0], -map_metadata["origin"][1]])
        / resolution
    ).astype(int)

    unfinished_grid_poses = find_existing_grid_poses(
        benchmark_poses_path=benchmark_poses_path,
        benchmark_range_data_path=benchmark_range_data_path,
    )
    if not unfinished_grid_poses:
        grid_poses = generate_grid_poses(
            map_metadata=map_metadata, free_map_img=free_binary_map, origin_px=origin_px
        )
        save_grid_poses(
            map_poses=grid_poses, benchmark_poses_path=benchmark_poses_path
        )
        unfinished_grid_poses = grid_poses

    synthesized_range_data = synthesize_ray_tracing_data(
        max_range_pixel=MAX_RANGE / resolution,
        map_poses=unfinished_grid_poses,
        free_map_img=free_binary_map,
        benchmark_range_data_path=benchmark_range_data_path,
        origin_px=origin_px,
    )
    visualize_map_with_range_data(
        synthesized_range_data,
        unfinished_grid_poses,
        map_image
    )
