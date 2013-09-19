cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(dviz_local)

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
  include/dviz_local/demonstration_visualizer_client.h 
)

# @todo there should be a better way to find necessary headers and libs
include_directories(../dviz_core/include)
link_directories(../dviz_core/lib)

rosbuild_add_executable(dviz_local
  src/demonstration_visualizer_client_node.cpp
  src/demonstration_visualizer_client.cpp
  ${RCC_FILES}
  ${MOC_FILES}
)

target_link_libraries(dviz_local
  dviz_core
  ${QT_LIBRARIES}
)
