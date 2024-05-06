## Rico's Implementation Of Google Cartographer
- Graph SLAM in 1D (D)
- Write about Graph SLAM in 1D (D)
- Graph SLAM in 2D (D)
- Write about Least Squares Implementation (D)
- [Install Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)
- Research 2D slam with loop closure
- Create a 3d point cloud simulation


### Videos
- [SLAM in 20min](https://www.youtube.com/watch?v=Alu59K8zvYs)
    - 3 paradigms: EKF; particle filtering; LS (graph slam, bundle adjustment)
    - In Graph Slam: 
        - a node is a pose estimate. An edge is the relative transform. (aka constraint). The problem is build the graph and find a node constraint that minimizes the error introduced by it. Each pose also has an observation (odom)
            - using observation from position x_i, what would the robot see at x_j?
        - On each edg, we generate a predicted view z'=f(x). Then single error vector e_i = (z'-z).  Squared Error is `se_i= e_i^TOe_i`. We want to minimize the sum of these errors: argmin sum()
        - Typically, we can calculate the derivative of the above and find its nulls. (over each Transform) But we need numerical methods to do that?
    - [UPenn Eng Overview Of Google Cartographer](https://www.youtube.com/watch?v=L51S2RVu-zc)
        - Brief History Of SLAM
        - Build a probablitstic grid map using odds 
    - [3d slam course on Bilibili](https://www.bilibili.com/video/BV1PB4y157mq/?spm_id_from=333.337.search-card.all.click)
### Papers
- Probablistic Robotics (Page 350, Table 11.1 - 11.5 for full algorithm)
- [A Tutorial On Graph Based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf), 2010
    - advancements in the field of sparse linear algebra resulted in efficient approaches 
    - Smoothing (graph) vs filtering  (particle / EKF)
    - front end vs back end: front end is constructing graph using sensor measurements. Backend is smoothing
- [Another good tutorial reading](https://essay.utwente.nl/71550/3/Appel_MA_EEMCS.pdf)
- [Google's Original Paper](https://storage.googleapis.com/gweb-research2023-media/pubtools/pdf/45466.pdf)
- GTSAM (Georgia Tech Smoothing and Maping) achieved good result, too. Works on sparse factor graph

### Implementations
- [Chenge Yang's Graph Slam Script](https://github.com/ChengeYang/Probabilistic-Robotics-Algorithms/blob/master/3.Graph_SLAM/Graph_SLAM_known_correspondences.py)


========================================================================
## Theory
========================================================================
1. Graph SLAM with known correspondences
    ```
    graph_slam_with_Known_Correspondences(u1:t, z1:t):
    initialize(u1:t):
        for t in range(T):
            xt = odom * xt-1
        x_0:t 
    while error > epsilon:
        linearize(states, u, z):
            info_matrix, info_vec
        reduce_map_features(info_matrix, info_vec)
        states = solve(info_matrix, info_vec)
    return states
    ```
    - Graph-Based Landmark SLAM: https://www.youtube.com/watch?v=mZBdPgBtrCM
    - Graph Based SLAM: https://www.youtube.com/watch?v=uHbRKvD8TWg&t=461s
    - Full course: http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/
        - should we create a LS solver first? 
2. There are visual slam and Lidar slam. Lidar slam started earlier, tech stack is more mature
    1. [Front end](http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/rob2-13-frontends.pdf)
        - ICP, feature based scan matching; descriptor based scan mactching.
        - Vanilla: apply ICP, evaulate the match. subject to sensitive initial guess, sampling. Ambiguities in env
        - So need local maps, and build global maps
            - SCGP: Sparse Constraints Graph Processing. It's sparse because each node is only connected to a few other nodes.
    2. Back end 
    3. Visual:
        - From 1 pair of 3D features, we can work out the best estimate of the transform
            - If you have a bunch of them: 
                1. Start from the best pair (pair that gives you the shortest descriptor distance)
                2. Reproject all features to image. 
                3. Do gradient descent on the params
        - Visual odometry is to find the trasform based on that.
    


### Detect Loop Closure
1. When to detect loop closures: 
    - every 10 scans;
    - if we are close to a previous location
2. If percentage of overllapping scan points is lower, then the candidate is rejected.

### Local SLAM and Global SLAM
1. optimize submap pose (relative to the global) and scan pose

========================================================================
## TODO
========================================================================
1. Market Research
    - How big is the demand of 3D nav?
        - Linked in Job post (gaia.ai, bear robotics?)
            - How large is the company?
        - Google Job post 
    - Ask people
2. Udacity Computer Vision Crash Course: https://www.udacity.com/course/computer-vision-nanodegree--nd891