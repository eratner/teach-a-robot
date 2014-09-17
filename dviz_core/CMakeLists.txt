cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(dviz_core)

set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# For Qt components.
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

rosbuild_add_library(dviz_core
  src/demonstration_visualizer_core.cpp
  src/demonstration_visualizer_user.cpp
  src/visualization_helpers.cpp
  src/demonstration_scene_manager.cpp
  src/object.cpp
  src/object_manager.cpp
  src/pr2_simulator.cpp
  src/base_carrot_controller.cpp
  src/end_effector_controller.cpp
  src/motion_recorder.cpp
  src/goal.cpp
  src/performance.cpp
)

target_link_libraries(dviz_core
  ${QT_LIBRARIES}
  sbpl_manipulation_components_pr2
)

rosbuild_add_executable(dviz_core_node
  src/demonstration_visualizer_core_node.cpp
)

rosbuild_add_executable(dviz_user_node
  src/demonstration_visualizer_user_node.cpp
)

rosbuild_add_executable(inspect_demonstration
  src/inspect_demonstration.cpp
)

target_link_libraries(dviz_core_node
  dviz_core
)

target_link_libraries(dviz_user_node
  dviz_core
)

target_link_libraries(inspect_demonstration
  dviz_core
)
