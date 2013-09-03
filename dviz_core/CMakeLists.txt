cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(dviz_core)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# For Qt components.
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

qt4_wrap_cpp(MOC_FILES 
  include/dviz_core/demonstration_visualizer.h 
)

rosbuild_add_library(dviz_core
  src/demonstration_visualizer.cpp
  src/visualization_helpers.cpp
  src/demonstration_scene_manager.cpp
  src/object.cpp
  src/object_manager.cpp
  src/pr2_simulator.cpp
  src/base_movement_controller.cpp
  src/end_effector_controller.cpp
  src/motion_recorder.cpp
  src/goal.cpp
  ${MOC_FILES}
)

target_link_libraries(dviz_core
  ${QT_LIBRARIES}
  sbpl_manipulation_components_pr2
)

rosbuild_add_executable(dviz_core_node
  src/demonstration_visualizer_node.cpp
)

target_link_libraries(dviz_core_node
  dviz_core
)
