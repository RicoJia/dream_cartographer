cmake_minimum_required(VERSION 3.10)
project(2DLocalizer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization and pthreads TODO: to recover add_compile_options(-O2)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Find packages
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(/usr/include/eigen3) # Include Eigen3

# Define the executable
add_executable(2DLocalizer 2dlocalizer.cpp)
# TODO - to delete
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Link libraries
target_link_libraries(2DLocalizer ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES}
                      Eigen3::Eigen Threads::Threads)

# find_package(CUDA) # Check if CUDA was found, flag from CMAKE if(CUDA_FOUND) #
# Attempt to find CUDA message(STATUS "CUDA Found: ${CUDA_VERSION_STRING}")
# enable_language(CUDA) include_directories(${CUDA_INCLUDE_DIRS})
# add_definitions(-DUSE_CUDA) # Specific CUDA configurations here... else()
# message(STATUS "CUDA not found. Skipping CUDA configuration.") endif()
