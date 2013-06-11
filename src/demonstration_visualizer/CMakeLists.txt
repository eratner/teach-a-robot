cmake_minimum_required(VERSION 2.8.3)
project(demonstration_visualizer)

## Find catkin macros and libraries.
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rviz
  pr2_motion_recorder
  visualization_msgs
  interactive_markers
  tf)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

catkin_package(
   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp rviz pr2_motion_recorder visualization_msgs interactive_markers tf
#   DEPENDS tinyxml
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(demonstration_visualizer
#   src/${PROJECT_NAME}/demonstration_visualizer.cpp
# )

qt4_wrap_cpp(MOC_FILES 
  include/demonstration_visualizer.h 
  include/demonstration_visualizer_node.h
)

add_executable(demo_vis 
  src/main.cpp 
  src/demonstration_visualizer.cpp
  src/demonstration_visualizer_node.cpp
  src/visualization_helpers.cpp
  src/base_movement_controller.cpp
  src/demonstration_scene_manager.cpp
  ${MOC_FILES}
)

link_directories(/home/eratner/pviz/lib)

target_link_libraries(demo_vis 
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  pviz
  tinyxml
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(demo_vis pviz pr2_motion_recorder_gencpp)
