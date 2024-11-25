cmake_minimum_required(VERSION 3.0.2)
project(rgbd_slam_rico)
add_compile_options(-std=c++2a)
# for vscode parameter clicking. Then
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # TODO: need to set to release mode
set(CMAKE_BUILD_TYPE Debug)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
  message("Using CCache")
endif()

set(CMAKE_PREFIX_PATH "$ENV{CUSTOM_INSTALL_PATH}" ${CMAKE_PREFIX_PATH})

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             # roslib is for ros::package::getPath()
             roslib
             rosbag
             sensor_msgs
             geometry_msgs
             cv_bridge # cv bridge stuff
             image_transport # For pub & sub video / compressed images
             pcl_ros
             pcl_conversions
             simple_robotics_ros_utils
             )
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g2o  REQUIRED)
find_package(GTest REQUIRED)
find_library(DL_LIB dl REQUIRED)
             
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv_bridge)

if(NOT DEFINED ENV{CUSTOM_INSTALL_PATH})
  message(FATAL_ERROR "CUSTOM_INSTALL_PATH environment variable is not set.")
endif()

set(SUITESPARSE_INCLUDE_DIR /usr/include/suitesparse)
set(SUITESPARSE_LIBRARY_DIR /usr/lib/x86_64-linux-gnu)
# Manually find the CSparse library
find_path(CSPARSE_INCLUDE_DIR cs.h PATHS ${SUITESPARSE_INCLUDE_DIR})
find_library(
  CSPARSE_LIBRARY
  NAMES cxsparse
  PATHS ${SUITESPARSE_LIBRARY_DIR})
if(NOT CSPARSE_INCLUDE_DIR OR NOT CSPARSE_LIBRARY)
  message(FATAL_ERROR "CSparse library not found")
else()
  message(NOTICE "Found cparse library")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${rosbag_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  $ENV{CUSTOM_INSTALL_PATH}/include
#   ${SIMPLE_ROBOTICS_CPP_UTILS_DIR}/include 
  ${GTest_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS} 
  include)

link_directories($ENV{CUSTOM_INSTALL_PATH}/lib)
add_library(${PROJECT_NAME}_dependencies INTERFACE)
target_link_libraries(
  ${PROJECT_NAME}_dependencies
    INTERFACE 
    ${catkin_LIBRARIES}
    ${rosbag_LIBRARIES}
    # simple_robotics_cpp_utils depend on OpenCV. So it MUST come before it
    simple_robotics_cpp_utils
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    ${CSPARSE_LIBRARY}
    Threads::Threads
    g2o::core
    g2o::stuff
    g2o::solver_dense
    g2o::types_slam2d
    g2o::types_slam3d
    g2o::solver_eigen
    g2o::types_sba
    g2o::csparse_extension
    ${DL_LIB}  # Add this line to link libdl
)

# Iterate through all cpp files in the folder
file(GLOB_RECURSE ALL_CPP_FILES "src/*cpp")

# Executables
foreach(CPP_FILE ${ALL_CPP_FILES})
    # use the filename without extension as the executable name
    get_filename_component(EXECUTABLE_NAME ${CPP_FILE} NAME_WE)
    add_executable(${EXECUTABLE_NAME} ${CPP_FILE})
    target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_dependencies)
endforeach()

# Test files
file(GLOB_RECURSE TEST_FILES "tests/unit/*cpp")
foreach(TEST_FILE ${TEST_FILES})
    # NAME_WE means "name without extension"
    get_filename_component(TEST_EXEC_NAME ${TEST_FILE} NAME_WE)
    catkin_add_gtest(${TEST_EXEC_NAME}_test ${TEST_FILE})
    target_link_libraries(${TEST_EXEC_NAME}_test
        # ${catkin_LIBRARIES}
        ${PROJECT_NAME}_dependencies
        # gtest
        # gtest_main
    )
endforeach()
enable_testing()