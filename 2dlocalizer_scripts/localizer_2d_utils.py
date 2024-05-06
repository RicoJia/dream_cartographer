import os
from copy import deepcopy
from enum import Enum
from typing import List

import cv2
import numpy as np
import yaml


class MapValue(Enum):
    UNKNOWN = 205
    INVALID = 1
    OCC = 0
    FREE = 254


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


def get_points_on_search_grid(
    map_image: np.ndarray,
    top_left: np.ndarray,
    bottom_right: np.ndarray,
    search_grid_resolution: int,
) -> np.ndarray:
    # top_left and bottom_right are in matrix coordinates
    search_grid_points = []
    for x in range(top_left[0], bottom_right[0], search_grid_resolution):
        for y in range(top_left[1], bottom_right[1], search_grid_resolution):
            if map_image[x, y] == MapValue.FREE.value:
                search_grid_points.append(np.array([x, y]))
    return np.asarray(search_grid_points).astype(int)


# def get_all_motions(search_grid_points_map_pixelized: List[np.ndarray]):
#     # Return points around
#     # optimization
#     NUM_ITERATIONS = 3
#     linear_step = 2**NUM_ITERATIONS
#     for point in search_grid_points_map_pixelized:
#         for i in range(NUM_ITERATIONS):
#             linear_step = int(linear_step/2)
#             motions += [
#                 # np.array([linear_step,0,0]),
#                 # np.array([0,linear_step,0]),
#                 # np.array([-linear_step,0,0]),
#                 # np.array([0,-linear_step,0]),
#                 # np.array([0,0,angle_step]),
#                 # np.array([0,0,-angle_step]),
#             ]


def get_vector_1_pixel_away_vectorized(starts: np.ndarray, end: np.ndarray):
    # starts: np.array([[2, 1], [2, 2], [2, 4]])
    vecs = end - starts
    norms = np.linalg.norm(vecs, axis=1, keepdims=True)
    norms[norms == 0] = 1  # prevent division by zero, or handle zero vectors as needed
    unit_vecs = np.round(vecs / norms).astype(int)
    return unit_vecs


###################################################################################
# map -> img Transforms
###################################################################################
def map_pixel_to_matrix_single(
    map_pixel: np.ndarray, origin_px: np.ndarray, img_height: float
):
    map_point = deepcopy(map_pixel)
    map_point += origin_px
    map_point[1] = img_height - map_point[1]
    map_point = map_point[::-1]
    return map_point


def map_pixel_to_matrix(
    points_for_all_thetas: List[np.ndarray], origin_px: np.ndarray, img_height: float
):
    # [theta1[np.array(x,y)...], theta2 [...]]
    # [10/resolution, 10/resolution] in m. This is the pixel coord of the map origin,
    # relative to the bottom left corner of the image
    # TODO: this actually messes up points_for_all_thetas
    return_arr = deepcopy(points_for_all_thetas)
    for points_for_single_theta in return_arr:
        points_for_single_theta += origin_px
        points_for_single_theta[:, 1] = img_height - points_for_single_theta[:, 1]
        points_for_single_theta[:, :] = points_for_single_theta[:, [1, 0]]
    return return_arr


def matrix_to_map_pixel(
    matrix_indices_orig: np.ndarray, origin_px: np.ndarray, img_height: float
):
    # [x1,y1; x2, y2...]
    matrix_indices = deepcopy(matrix_indices_orig)
    matrix_indices = matrix_indices[:, [1, 0]]
    matrix_indices[:, 1] = img_height - matrix_indices[:, 1]
    matrix_indices -= origin_px
    return matrix_indices.astype(int)


def matrix_coords_to_image_coords(matrix_coords):
    return matrix_coords[::-1]


def map_to_img_pixel(map_point: np.ndarray, origin_px: np.ndarray, img_height: float):
    matrix_coords = map_pixel_to_matrix_single(map_point, origin_px, img_height)
    return matrix_coords_to_image_coords(matrix_coords)


###################################################################################
# img -> map Transforms
###################################################################################
def img_pixel_to_matrix_coords(image_coords):
    return image_coords[::-1]


def matrix_coords_to_shifted_map(matrix_coords, img_height):
    return np.array([matrix_coords[1], img_height - matrix_coords[0]])


def shifted_map_to_map(shifted_map_point, origin_xy_pixel):
    return shifted_map_point - origin_xy_pixel


def img_pixel_to_map(image_coords, img_height, origin_px):
    matrix_coords = img_pixel_to_matrix_coords(image_coords)
    shifted_map_point = matrix_coords_to_shifted_map(matrix_coords, img_height)
    return shifted_map_to_map(shifted_map_point, origin_px)


###################################################################################
# Img utils
###################################################################################
def add_pose_to_relative_poses(
    p_hits_for_all_thetas: List[np.ndarray], pose: np.ndarray
):
    # pose is [x,y]
    p_hits_for_all_thetas_single_cp = deepcopy(p_hits_for_all_thetas)
    for p_hits_for_all_thetas_single in p_hits_for_all_thetas_single_cp:
        p_hits_for_all_thetas_single += pose
    return p_hits_for_all_thetas_single_cp


def create_mask(p_hits, img_width, img_height):
    xor_mask = MapValue.INVALID.value * np.ones((img_height, img_width))
    xor_mask[p_hits[:, 0], p_hits[:, 1]] = MapValue.OCC.value
    # TODO
    # xor_mask[p_frees[:, 0], p_frees[:, 1]] = MapValue.FREE.value
    return xor_mask


def get_binary_image(image):
    return (image > 0).astype(np.uint8)


def get_gradient_mat(mat):
    sobelx = cv2.Sobel(mat, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(mat, cv2.CV_64F, 0, 1, ksize=3)
    result_img = np.sqrt(sobelx**2 + sobely**2)
    result_img = cv2.GaussianBlur(
        result_img, (1, 1), 0
    )  # (5, 5) is the kernel size, 0 is sigmaX
    return get_binary_image(result_img)


if __name__ == "__main__":
    starts = np.array([[2, 1], [2, 2], [2, 4]])
    get_vector_1_pixel_away_vectorized(starts, np.array([0, 0]))

    test_points_for_all_thetas = [np.array([[1, 1], [2, 2], [5, 6]])]
    arr = map_pixel_to_matrix(test_points_for_all_thetas, np.array([1, 1]), 10)
    # assert np.array_equal(arr[0], np.array([[8,2], [7,3], [3,6]]))
    print(f"map to pixel: {arr}")

    arr = matrix_to_map_pixel(arr[0], np.array([1, 1]), img_height=10)
    print(f"matrix to pixel: {arr}")

    print("gradient: ", get_gradient_mat(np.identity(3)))
