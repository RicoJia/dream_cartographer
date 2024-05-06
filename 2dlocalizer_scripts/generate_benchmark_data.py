#!/usr/bin/env python3
"""
1. Take in an image
2. Get a group of grid points on the image, if the point is free
3. For a given max_range, 
    1. The orientation of the robot is randomly generated
    2. Sythesize ray tracing data. If there are points outside of the range, inf will be returned
4. Save the range data, and the pose into a csv file. |x, y, theta, range_data| 
"""

import argparse
import os
from collections import deque

import numpy as np
from localizer_2d_utils import (MapValue, get_file_path_in_the_same_folder,
                                img_pixel_to_map, load_map_and_meta_data,
                                map_to_img_pixel, rgba_to_grayscale)

# Must be integer, this is relative to the map resolution
GRID_STEP_MULTIPLIER = 3
SAVED_DATA_FOLDER = "data"
SAVED_DATA_FILE = "benchmark_data.csv"


def get_img_and_metadata_path():
    parser = argparse.ArgumentParser()
    # TODO
    parser.add_argument("--img_name", type=str, default="map499.png")
    # parser.add_argument("--img_name", type=str, required=True, default="map499.png")
    args = parser.parse_args()
    img_path = os.path.join(SAVED_DATA_FOLDER, args.img_name)
    metadata_path = ".".join(img_path.split(".")[:-1]) + ".yaml"
    return img_path, metadata_path


def generate_grid_poses(map_metadata, map_image, origin_px):
    """Return the map pixel poses on a grid that are free pixels"""
    # the grid is like a checker board
    height, width = map_image.shape
    free_thresh, negate = map_metadata["free_thresh"], bool(map_metadata["negate"])
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

    map_poses = deque()
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


def save_data():
    pass


if __name__ == "__main__":
    img_path, metadata_path = get_img_and_metadata_path()
    map_metadata, map_image = load_map_and_meta_data(
        img_path=img_path, metadata_path=metadata_path
    )
    map_image = rgba_to_grayscale(map_image)
    resolution = map_metadata["resolution"]
    origin_px = (
        np.array([-map_metadata["origin"][0], -map_metadata["origin"][1]]) / resolution
    ).astype(int)

    grid_poses = generate_grid_poses(
        map_metadata=map_metadata, map_image=map_image, origin_px=origin_px
    )
