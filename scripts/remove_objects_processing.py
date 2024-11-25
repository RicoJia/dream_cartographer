#!/usr/bin/env python3
from ricomodels.deeplabv3_plus.predict_deeplabv3_plus import PredictBench
from torchvision import models
from typing import List
from tqdm import tqdm
import numpy as np
import os

from cv_bridge import CvBridge
import rosbag

if __name__ == "__main__":
    
    # TODO: can make this a context manager
    WORKSPACE_PATH = os.path.dirname(os.path.abspath(os.getcwd()))
    ROSBAG_PATH = os.path.join(WORKSPACE_PATH, "rgbd_slam_rico/data", "rgbd_dataset_freiburg2_desk_with_person.bag")
    OUTPUT_ROSBAG_PATH = ROSBAG_PATH+".processed"
    print(f'Rico: {OUTPUT_ROSBAG_PATH}')

    RGB_TOPIC = "/camera/rgb/image_raw"
    bridge = CvBridge()
    with rosbag.Bag(ROSBAG_PATH, "r") as in_bag, rosbag.Bag(OUTPUT_ROSBAG_PATH, "w") as out_bag:
        for topic, msg, t in in_bag.read_messages():
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(f'Rico: {topic}')
    
    # bench = PredictBench(
    #     # aux_loss: If True, include an auxiliary classifier
    #     model = models.segmentation.deeplabv3_resnet101(pretrained=True)
    # )
    # outputs = bench.predict([image])
    # for output_batch in outputs:
