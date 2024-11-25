#!/bin/bash

# README: if you change the content of this file, please rebuild the container

echo "Running Docker entry point..."
# .${SIMPLE_ROBOTICS_UTILS_DIR}/simple_robotics_cpp_utils/
${SIMPLE_ROBOTICS_UTILS_DIR}/simple_robotics_cpp_utils/build.sh

exec "$@"