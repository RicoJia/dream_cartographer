cmake_minimum_required(VERSION 3.10)
project(DreamCartographerExercises)

# Set C++17 as the standard for the project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(g2o REQUIRED)
find_package(Ceres REQUIRED)
find_library(GLOG_LIBRARY glog)
find_path(GLOG_INCLUDE_DIR glog/logging.h)
include_directories(${GLOG_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executable target
add_executable(simple_g2o_example simple_g2o_example.cpp)

# Link the libraries
target_link_libraries(
  simple_g2o_example
  g2o::core
  g2o::stuff
  g2o::solver_dense
  g2o::types_slam2d
  g2o::types_slam3d
  g2o::solver_eigen
  g2o::types_sba)

add_executable(handcrafted_gauss_newton_opt handcrafted_gauss_newton_opt.cpp)
target_link_libraries(handcrafted_gauss_newton_opt ceres ${GLOG_LIBRARY})
