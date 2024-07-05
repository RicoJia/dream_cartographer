# Survey On Lidar SLAM

What does SLAM do?

Man, SLAM is hard. There has been research on it for 40+ years, but there are not so many in-depth but also beginner-resources on it. 



## Graph Slam
[Nice video](https://www.bilibili.com/video/BV1ut4y1X7f7/?spm_id_from=333.337.search-card.all.click)
Construct a graph:
1. A node is a pose with observation (Lidar, image, depth image)
2. An edge is the homogeneous transform between 2 nodes



https://www.cnblogs.com/gaoxiang12/p/3695962.html



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

### General Graph Optimization (G2O) for SLAM/Bundle Adjustment Least Squares Graph Optimization
[Paper](https://mengwenhe-cmu.github.io/Reading-Reports/Research/Localization/Graph_Optimization/g2o_A_General_Framework_for_Graph_Optimization/paper.pdf)
[video](https://www.bilibili.com/video/BV1Ca41167Sb/?spm_id_from=333.337.search-card.all.click)

We are using Gauss Newton / Levenberg-Marquardt optimization to carry out the optimization. These optimizations:
- Are good for convex functions. So you might not find the global minimum
- So they rely on the initial values of the parameters

Implementation Example: [Little SLAM](https://github.com/furo-org/LittleSLAM/tree/master/framework)

#### How to calculate Jacobian

- Finite difference method

### Research Papers
1. Karto: 
    - Karto has a loop closure detection, and uses sparse matrix to solve. It can be used in large environment mapping
    - Repos:    
        - [slam karto, the backend](https://github.com/ros-perception/slam_karto/tree/melodic-devel)
        - [Open Karto, the ros package](https://github.com/ros-perception/open_karto)
    - Papers:
        - [Real-Time Correlative Scan Matching](https://april.eecs.umich.edu/pdfs/olson2009icra.pdf)
        - [Efficient Sparse Pose Adjustment for 2D Mapping](http://ais.informatik.uni-freiburg.de/publications/papers/konolige10iros.pdf)

2. Cartographer TODO
    - [Mandarin Code Analysis](https://mp.weixin.qq.com/s/LdbFp-Zvkr02-_25ILb16g?)

3. Gaoxiang Fan Club
    - [高翔code](https://github.com/gaoxiang12/rgbd-slam-tutorial-gx/blob/master/part%20VII/src/detectFeatures.cpp)
    - [Blog](https://www.cnblogs.com/gaoxiang12/p/4659805.html)

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


### ICP
PCL - Generalized Iterative Closest Point (G-ICP) Method
icp(inital_guess, cloud1, cloud2)
Start the algorithm with an initial guess. In SLAM, you can use odom information as one.
Iteration starts
Find Matching Point pairs between the two point clouds. using kd-tree
Estimate Pose using Least Squares.
Check for errors. If error is still significant, go back to step 2. Otherwise stop

Step 3 is really key here. Here is how it goes
objective: minimize sum(||R*p1+T-p2||²).Where we have rotation matrix R and translation T for the transform

### ALOAM (great for beginners)

Intro
    - [Repo](https://gitcode.com/HKUST-Aerial-Robotics/A-LOAM/overview?utm_source=csdn_github_accelerator)
    - When?

Advantages:
- Compared to LOAM: euler angles are changed to quaternion; 
- Using Ceres

### HDL Graph SLAM
Intro: Simple graph based 3D lidar slam. In the front end, you can use ICP, NDT. 
    - [Repo](https://gitcode.com/koide3/hdl_graph_slam/overview?utm_source=csdn_github_accelerator)

## 3D Visual SLAM

### RGBD SLAM2 (Beginner Friendly， 2014)
Resources:
    - [Paper: Endres et al, 3D Mapping with an RGB-D camera, TRO, 2014.](http://www2.informatik.uni-freiburg.de/~endres/files/publications/endres13tro.pdf)
    - [Repo](https://github.com/felixendres/rgbdslam_v2?tab=readme-ov-file)
    - [Website](http://felixendres.github.io/rgbdslam_v2/)
Intro
    - see rgbd-slam-ros, 视觉slam 实战。There's feature extraction, optimization, loop detection, 3d point cloud, octomap, 非常全面. 
    - Disadvantages:
        - Using SIFT features (very slow), rendering pointcloud, so realtimeness sucks, and map building could get stuck.
        - Key frame sampling frequency is high


### GCN SLAM

orb features vs GCN (deep learned) features
    - When there's a lot of texture, ORB can perform well. When images change fast, ORB features will "cluster", while GCN features 
    are still evenly distributed. [Video](https://www.bilibili.com/video/BV1ei4y1F7HV/?spm_id_from=333.337.search-card.all.click)

### ORB-SLAM

ORB-SLAM uses ORB features as feature detector and descriptor, which is robust against rotation and scale changes.
A map is composed of keypoints and key frames.Also, it uses bag-of-words. 
    - ORB-SLAM 1 by Tardos et al. in 2015. It's focused on robust SLAM solution
    - ORB-SLAM 2 can work with stereo and RGBD camera in 2016
    - ORB-SLAM 3 can work with multi-map stitching and visual inertial data.

### RTAB-MAP
#### Intro 
Resources:
    - [paper](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf)
    - [Principle (Mandarin)](https://blog.csdn.net/xmy306538517/article/details/78771104)
    - [overview article](https://shivachandrachary.medium.com/introduction-to-3d-slam-with-rtab-map-8df39da2d293)
    - [rtab map ros](https://github.com/introlab/rtabmap_ros)
    - [Good Mandarin ](https://blog.51cto.com/remyspot/1784914)

Advantages:
    - Advantage: precise.
    - Advantage: Compared to ORB-SLAM, the map is denser. 

Disadvantages:
    - Disadvantage: long map building time & light changes could affect re-localization
    - Disadvantage: Using possion for point cloud voxelization, instead of TSDF (mainstream)?

Inputs:
    - TF between sensor and base
    - Odom info
    - Camera input (RGBD, or stereo) with calibration

Outputs:
    - Octomap
    - Occupancy map

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

## Algorithm
1. `create_location(image) -> L_t`
    - Use surf to extract features. If there are enough features, rejuect (like a wall)
    - Feature vector space will be incremental, that is, we don't need to pre-train. A vector is aka **signature**
        - Use K-means to create new features. If you can find similar features, then the new feature can be aggregated into one? [Question]
            - Yep, nearest neighbor distance ratio = $R=\frac{d_1}{d_2}$ where 1 is to the nearest neighbor, 2 is to the second nearest neighbor. If a new feature has NNDR lower than a threshold, then the feature is distinct and can be represented by it
            - New feature is added when NNDR is below a ratio? **Could it be ambiguous features??**
        - An image is a K-vector, where each element is a count

2. Insert `L_(t)` into the STM
    - Add a bi-directional link to previous time, $L_c$
    - Similarity $s = N_{pair}/max(N_{zt}, N_{zc})$, where N is number of features. $N_{pair}$ is the common pairs of features. 
    - If similar, $Z_c$ will be migrated into $Z_t$? $L_c$ will be deleted, and $L_t $weight will be $L_c+1$

3. If STM size has reached threshold, 
    - Move the oldest to WM

4. Check for loop closure probablity $p(S_t|L_t)$ among $S_t$ in Working Memory (this is done in Bayesian Filter)
    - If $S_t$ is found, built a loop closure link

5. Join trash's thread? 
6. Memory mgmt:
    - Retrieval (L_i) LTM -> WM
    - If $Image_processing_time > Thre$
        - Transfer(WM->LTM), lowest weight, and oldest
    

## Appendix

### SQLite Database
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
    
References:
 Gao Xiang A story on SLAM: https://www.cnblogs.com/gaoxiang12/p/3695962.html