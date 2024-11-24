#!/usr/bin/env python3
from ricomodels.deeplabv3_plus.predict_deeplabv3_plus import PredictBench
from torchvision import models
from typing import List
from tqdm import tqdm
import numpy as np
import os

if __name__ == "__main__":
    
    WORKSPACE_PATH = os.path.dirname(os.path.abspath(os.getcwd()))
    ROSBAG_PATH = os.path.join(WORKSPACE_PATH, "rgbd")
    #TODO Remember to remove
    print(f'Rico: {ROSBAG_PATH}')
    
    # bench = PredictBench(
    #     # aux_loss: If True, include an auxiliary classifier
    #     model = models.segmentation.deeplabv3_resnet101(pretrained=True)
    # )
    # outputs = bench.predict([image])
    # for output_batch in outputs:
