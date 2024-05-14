# Survey On Lidar SLAM

## Graph Slam
[Nice video](https://www.bilibili.com/video/BV1ut4y1X7f7/?spm_id_from=333.337.search-card.all.click)
Construct a graph:
1. A node is a pose with observation (Lidar, image, depth image)
2. An edge is the homogeneous transform between 2 nodes

### Goal
- optimize all edges such that when a loop closure is detected, the loop node and the start of the graph 
will be the same. That is, the loop->start Transform should be Identity transform. Each transform is like
a spring. The more confident you are, the more stiff it is. 

<p align="center">
<img src="https://github.com/RicoJia/The-Dream-Robot/assets/39393023/268b8480-5cd7-4606-847d-1c3b5bf84d98" height="400" width="width"/>
<figcaption align="center">Optimize Edges So The Last Edge Disappears!</figcaption>
</p>

So if you have a bad loop association, it will be bad. **Better miss a loop closure, than having a bad one**

Also, edges of the current sliding window will be considered for optimization

### G2O A General Framework for Graph Optimization
[Paper](https://mengwenhe-cmu.github.io/Reading-Reports/Research/Localization/Graph_Optimization/g2o_A_General_Framework_for_Graph_Optimization/paper.pdf)
[video](https://www.bilibili.com/video/BV1Ca41167Sb/?spm_id_from=333.337.search-card.all.click)

We are using Gauss Newton / Levenberg-Marquardt optimization to carry out the optimization. These optimizations:
- Are good for convex functions. So you might not find the global minimum
- So they rely on the initial values of the parameters

Implementation Example: [Little SLAM](https://github.com/furo-org/LittleSLAM/tree/master/framework)

#### How to calculate Jacobian 
- Finite difference method
    ```


    
    ```

## 3D Lidar SLAM

- How does tesla build its low occupancy map?
    - Some of the foundamental challenges
    - Mislocalization (Lidar hijacking)
    - Else?
    - FSD (since March 2023) is "vision only"
        - Traditionally, builds HD map (High Definition)
        - When radar and vision disagrees, what do you do? Doubling down on camera, because it's so much more accurate

    - They want to estimate how fast pedestrians are travelling at, a supervised learning problem.
        - Large dataset (millions of videos)
        - Clean (labeled data, depth)
        - Edge cases: not just nominal cases

ALOAM
Cartographer

## 3D Visual SLAM
### ORB SLAM; GCN SLAM
- orb features vs GCN (deep learned) features
    - When there's a lot of texture, ORB can perform well. When images change fast, ORB features will "cluster", while GCN features 
    are still evenly distributed. [Video](https://www.bilibili.com/video/BV1ei4y1F7HV/?spm_id_from=333.337.search-card.all.click)
### RTAB-MAP
[paper](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf)
[overview article](https://shivachandrachary.medium.com/introduction-to-3d-slam-with-rtab-map-8df39da2d293)
[rtab map ros](https://github.com/introlab/rtabmap_ros)
Compared to ORB-SLAM, the map is denser. 
    - Multi-session mapping so it could recognize previously mapped areas and continue mapping; RTAB-MAP uses a SQLite database. [Paper](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/e/eb/Labbe14-IROS.pdf)
    - Visual Loop Closure Detection:
        1. Extract features, have an index for each of these, then convert the image into a vector.
        2. Put them in a vocabulary tree
        3. Query the database and find the most similar vector
            - Match features the image with the candidate image.
    - 3D point cloud generation:
        - Question: is there any processing on these point cloud generation?

- How to set up:
    - https://dev.to/admantium/ros-simultaneous-mapping-and-localization-with-rtabmap-2o09
    - SLAM Project: https://github.com/Robotics-lessons/term2-slam-project


#### SQLite Database
SQLite Database is chosen because it's disk like, and no separate process is run; It could handle read-write more efficiently? 
    - Multiple processes can access the same database file directly at the same time
    - It's a classic "serverless" structure, meaning, the database engine runs in the same process, same thread as the application. 
    - Neo serverless databases: - server runs on a separate machine. Example: Amazon S3. The database server do exist, but is hidden from the users.
    - Advantages of with-server database:
        - "A stray pointer" (pointer accessing uninitialized memory) cannot corrupt the database"

Use cases:
    - Not intended for multiple routes to access the same database at the same time. It's for having multiple local instances of the database without human support, such as IoT, robots, etc.
    - Medium to low traffic (100k hits/day, less than 670 hits per second) is fine

Other facts:
    - Written in C, most widely used database engine: found in safari, firefox, chrome. Started in 2000,
    

### ORB-SLAM
ORB-SLAM uses ORB features as feature detector and descriptor, which is robust against rotation and scale changes.
A map is composed of keypoints and key frames.Also, it uses bag-of-words. 
    - ORB-SLAM 1 by Tardos et al. in 2015. It's focused on robust SLAM solution
    - ORB-SLAM 2 can work with stereo and RGBD camera in 2016
    - ORB-SLAM 3 can work with multi-map stitching and visual inertial data.
