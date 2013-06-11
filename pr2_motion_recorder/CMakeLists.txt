cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(pr2_motion_recorder)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Generate service files in the srv directory.
rosbuild_gensrv()

rosbuild_add_executable(pr2_motion_rec 
  src/main.cpp
  src/pr2_motion_recorder.cpp
)