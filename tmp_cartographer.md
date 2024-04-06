## Rico's Implementation Of Google Cartographer
- Graph SLAM in 1D 
- Write about Graph SLAM in 1D 
- Graph SLAM in 2D 
- Least Squares Implementation
- Write about Least Squares Implementation

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
### Papers
- Probablistic Robotics (Page 350, Table 11.1 - 11.5 for full algorithm)
- [A Tutorial On Graph Based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf), 2010
    - advancements in the field of sparse linear algebra resulted in efficient approaches 
    - Smoothing (graph) vs filtering  (particle / EKF)
    - front end vs back end: front end is constructing graph using sensor measurements. Backend is smoothing
- [Another good tutorial reading](https://essay.utwente.nl/71550/3/Appel_MA_EEMCS.pdf)
- [Google's Original Paper](https://storage.googleapis.com/gweb-research2023-media/pubtools/pdf/45466.pdf)

### Implementations
- [Chenge Yang's Graph Slam Script](https://github.com/ChengeYang/Probabilistic-Robotics-Algorithms/blob/master/3.Graph_SLAM/Graph_SLAM_known_correspondences.py)


========================================================================
## Theory
========================================================================
1. what is a non euclidean space?