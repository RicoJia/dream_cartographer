cmake_minimum_required(VERSION 3.0.2)
project(rgbd_slam_rico)
add_compile_options(-std=c++2a)
# for vscode parameter clicking. Then
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             # roslib is for ros::package::getPath()
             roslib
             simple_robotics_ros_utils
             rosbag
             sensor_msgs
             cv_bridge # cv bridge stuff
             # image_transport # For pub & sub video / compressed images
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv_bridge)
set(SIMPLE_ROBOTICS_CPP_UTILS_DIR
    ${CMAKE_SOURCE_DIR}/../../simple_robotics_cpp_utils)
add_subdirectory(${SIMPLE_ROBOTICS_CPP_UTILS_DIR}
                 ${CMAKE_BINARY_DIR}/simple_robotics_cpp_utils)

add_library(${PROJECT_NAME}_dependencies INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}_dependencies
  INTERFACE ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${OpenCV_LIBRARIES}
            simple_robotics_cpp_utils Eigen3::Eigen)

include_directories(
  ${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${SIMPLE_ROBOTICS_CPP_UTILS_DIR}/include include)
add_executable(orb_test src/orb_test.cpp)
target_link_libraries(orb_test ${PROJECT_NAME}_dependencies)

add_executable(rgbd_rico_slam_mini src/rgbd_rico_slam_mini.cpp)
target_link_libraries(rgbd_rico_slam_mini ${PROJECT_NAME}_dependencies)
