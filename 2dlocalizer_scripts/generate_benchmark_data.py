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

# Must be integer, this is relative to the map resolution
GRID_STEP_MULTIPLIER = 3
SAVED_DATA_FOLDER = "data"
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


def generate_grid_poses(
    map_metadata, map_image, origin_px
) -> List[np.ndarray]:
    """Return the map pixel poses on a grid that are free pixels"""
    # the grid is like a checker board
    height, width = map_image.shape
    free_thresh, negate = map_metadata["free_thresh"], bool(
        map_metadata["negate"]
    )
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

    map_poses = list()
    for hx in range(GRID_STEP_MULTIPLIER, height, GRID_STEP_MULTIPLIER):
        for hy in range(GRID_STEP_MULTIPLIER, width, GRID_STEP_MULTIPLIER):
            if free_map_img[hx, hy] == True:
                map_pixel_point = img_pixel_to_map(
                    image_coords=np.array((hx, hy)),
                    img_height=height,
                    origin_px=origin_px,
                )
                theta = np.random.uniform(0, 2 * np.pi)
                map_poses.append(np.append(map_pixel_point, [theta]))
    return map_poses


def synthesize_ray_tracing_data(
    max_range_pixel: np.ndarray,
    map_poses: List[np.ndarray],
    map_image: np.ndarray,
    benchmark_range_data_path: str,
):
    # [x, y, theta], theta is relative to the orthogonal map pose
    for pose in map_poses:
        pass
        # TODO: save simulated range data


def save_grid_poses(map_poses, benchmark_poses_path):
    with open(benchmark_poses_path, "w") as poses_file:
        writer = csv.writer(poses_file)
        writer.writerow(["x", "y", "theta"])
        for row in map_poses:
            writer.writerow(row)
    # TODO Remember to remove
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
            poses.append(np.array([row["x"], row["y"], row["theta"]]))

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


if __name__ == "__main__":
    (
        img_path,
        metadata_path,
        benchmark_poses_path,
        benchmark_range_data_path,
    ) = get_paths()
    map_metadata, map_image = load_map_and_meta_data(
        img_path=img_path, metadata_path=metadata_path
    )
    map_image = rgba_to_grayscale(map_image)
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
            map_metadata=map_metadata, map_image=map_image, origin_px=origin_px
        )
        save_grid_poses(
            map_poses=grid_poses, benchmark_poses_path=benchmark_poses_path
        )
        unfinished_grid_poses = grid_poses

    synthesized_range_data = synthesize_ray_tracing_data(
        max_range_pixel=MAX_RANGE / resolution,
        map_poses=unfinished_grid_poses,
        map_image=map_image,
        benchmark_range_data_path=benchmark_range_data_path,
    )
