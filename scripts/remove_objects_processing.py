#!/usr/bin/env python3
from ricomodels.deeplabv3_plus.predict_deeplabv3_plus import PredictBench

from torchvision import models
from typing import List
from tqdm import tqdm
import numpy as np
import os
import sys

from simple_robotics_python_utils.common.ros1bag_ros2bag_conversions import (
    read_ros1_do_stuff_save_ros1,
)
import numpy as np
import functools
import matplotlib.pyplot as plt


def post_process_image(msg, bench, visualize):
    data = msg.data
    height = msg.height
    width = msg.width
    encoding = msg.encoding
    if encoding == "rgb8":
        # image = np.frombuffer(data, np.uint8).reshape((height, width, 3))
        image = np.frombuffer(data, np.uint8).reshape((height, width, 3))
    elif encoding == "bgr8":
        image = np.frombuffer(data, np.uint8).reshape((height, width, 3))
    elif encoding == "mono8":
        image = np.frombuffer(data, np.uint8).reshape((height, width))
    else:
        raise NotImplementedError(f"Encoding {encoding} not supported.")
    output, masked_output = bench.predict([image], masked_classes=["person"])
    output, masked_output = output[0], masked_output[0]

    # Handle channel swapping if encoding is 'bgr8'
    if encoding == "bgr8" and masked_output.ndim == 3:
        masked_output = masked_output[:, :, ::-1]

    # Ensure the masked_output has the correct dtype and range
    if masked_output.dtype != np.uint8:
        # Assuming masked_output is a binary mask or probabilities, scale appropriately
        masked_output = (masked_output * 255).astype(np.uint8)

    # Handle different encodings
    if encoding in ["rgb8", "bgr8"]:
        # Ensure masked_output has shape (H, W, 3)
        if masked_output.ndim == 2:
            # If mask is single channel, stack to make it 3-channel
            masked_output = np.stack([masked_output] * 3, axis=-1)
        elif masked_output.shape[2] == 1:
            masked_output = np.concatenate([masked_output] * 3, axis=2)
    elif encoding == "mono8":
        # Ensure masked_output has shape (H, W)
        if masked_output.ndim == 3:
            masked_output = masked_output.squeeze(2)

    # Assign the processed mask back to msg.data as a NumPy array
    msg.data = masked_output.flatten()  # Or use masked_output.ravel()

    if visualize:
        plt.subplot(1, 3, 1)
        plt.title("Input Image")
        plt.imshow(image)
        plt.subplot(1, 3, 2)
        plt.title("Mask Prediction")
        plt.imshow(output)
        plt.subplot(1, 3, 3)
        plt.title("Masked Output")
        plt.imshow(masked_output)
        plt.show()
    return msg


if __name__ == "__main__":

    # Python 3.10 check
    if sys.version_info.major == 3 and sys.version_info.minor == 10:
        print(f"Python 3.10 detected, great! 😊")
    else:
        print(f"Python 3.10 is required, aborted, sorry! 😭")

    # TODO: can make this a context manager
    WORKSPACE_PATH = os.path.dirname(os.path.abspath(os.getcwd()))
    input_path = os.path.join(
        WORKSPACE_PATH, "rgbd_slam_rico/data", "rgbd_dataset_freiburg1_xyz.bag"
    )
    output_path = input_path + ".processed"
    print(f"Rico: {output_path}")

    RGB_TOPIC = "/camera/rgb/image_raw"

    bench = PredictBench(model=models.segmentation.deeplabv3_resnet101(pretrained=True))
    read_ros1_do_stuff_save_ros1(
        input_path=input_path,
        output_path=output_path,
        do_stuff={
            "/camera/rgb/image_color": functools.partial(
                post_process_image, bench=bench, visualize=False
            )
        },
    )
