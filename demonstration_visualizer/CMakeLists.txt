cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(demonstration_visualizer)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_genmsg()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# For Qt components.
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

qt4_add_resources(RCC_FILES
  icons.qrc
)

qt4_wrap_cpp(MOC_FILES 
  include/demonstration_visualizer/demonstration_visualizer.h 
  include/demonstration_visualizer/demonstration_visualizer_node.h
)

rosbuild_add_executable(dviz
  src/main.cpp
  src/demonstration_visualizer.cpp
  src/demonstration_visualizer_node.cpp
  src/visualization_helpers.cpp
  src/demonstration_scene_manager.cpp
  src/object.cpp
  src/object_manager.cpp
  src/pr2_simulator.cpp
  src/base_movement_controller.cpp
  src/end_effector_controller.cpp
  src/motion_recorder.cpp
  src/goal.cpp
  ${RCC_FILES}
  ${MOC_FILES}
)

target_link_libraries(dviz 
  ${QT_LIBRARIES}
  sbpl_manipulation_components_pr2
)
