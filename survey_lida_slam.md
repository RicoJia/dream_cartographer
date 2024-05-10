# Survey On Lidar SLAM

## 2D LIDAR SLAM
- gmapping
- 
## 3D Lidar SLAM
ALOAM
Cartographer

## 3D Visual SLAM
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
