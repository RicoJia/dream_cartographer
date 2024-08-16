## RGBD SLAM Exercises

This directory is a group of exercises for better understanding major steps in RGBD SLAM. 

### ORB From Scratch

I wrote unit tests for all major functions for ORB Detection, descriptor generation, and descriptor(feature) matching. To launch all associated tests:

```bash
catkin build
rosrun rgbd_slam_rico test_orb_test
# hit <Esc> to close each image
```

Due to the sake of time constraint, some future work would be nice for enhancing this custom implementation. However, I did some correctness tests:

#### Test result

- custom ORB + custom feature matching

<div style="text-align: center;">
<p align="center">
    <figure>
        <img src="https://github.com/user-attachments/assets/e504a1ff-7625-4362-87c1-b64c41ac55b1" height="200" alt=""/>
    </figure>
</p>
</div>

- custom ORB + CV brute force KNN feature matching

<div style="text-align: center;">
<p align="center">
    <figure>
        <img src="https://github.com/user-attachments/assets/5d80f3a9-5f62-4235-9fcb-6098af2c8f49" height="200" alt=""/>
    </figure>
</p>
</div>

- CV ORB + custom feature matching

<div style="text-align: center;">
<p align="center">
    <figure>
        <img src="https://github.com/user-attachments/assets/9d5eb561-d944-4ec0-9647-8c495bcc7f7c" height="200" alt=""/>
    </figure>
</p>
</div>

- CV ORB + CV brute force KNN feature matching

<div style="text-align: center;">
<p align="center">
    <figure>
        <img src="https://github.com/user-attachments/assets/dd83f256-3d15-4fec-ab36-7a004a20dfe6" height="200" alt=""/>
    </figure>
</p>
</div>

As can be seen, there are certain flaws and highlights in my custom feature detection:

- In ORB feature detection
    - Some feature points are sub-optimal, especially the ones on the door frame. As of Aug 15, 2024, this is probably because my implementation currently only works with 1-level image pyramid. It could generate keypoints from different levels, but it does not yet merge them together. 
    - One highlight is that the keypoints are evaluated with Harris corner responses, same as [the OpenCV implementation](https://github.com/barak/opencv/blob/051e6bb8f6641e2be38ae3051d9079c0c6d5fdd4/modules/features2d/src/orb.cpp#L693).
    - Another highlight is that to calculate the feature's orientation, the custom implementation calculates the image moment of a circular patch using the integral image of the original image. This gives a better & faster estimate than finding orientation over a rectangular patch.

- For Feature Descriptor Generation and matching,
    - One TODO is probably to generate a descriptor for custom features. We are currently utilizing `orb->detectAndCompute(image, cv::noArray(), res.keypoints, res.descriptor)` so we are not generating the descriptor separately.
    - The custom descriptor generation and matching should be double checked, as indicated by the last two tests above. 
    - One small highlight of descriptor generation is the discretization of rotation matrices (as indicated by the original ORB paper). That theoretically should speed things up.