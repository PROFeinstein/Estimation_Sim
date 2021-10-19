cmake_minimum_required(VERSION 2.8.3)
project(tgt_pstion)

add_definitions(-std=c++11)

#set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  pelican_ctrl_pkg
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories( include ../pelican_ctrl_pkg/include ../tgt_pstion/include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs image_geometry pelican_ctrl_pkg detection_network
  DEPENDS system_lib opencv Eigen
)

###############################################################################
add_library(tgt_kalman_filters src/library/tgt_kalman_filters.cpp)
target_link_libraries(tgt_kalman_filters ${catkin_LIBRARIES})
add_dependencies(tgt_kalman_filters ${catkin_EXPORTED_TARGETS})
###############################################################################
add_library(target_position_estimation src/library/target_position_estimation.cpp)
target_link_libraries(target_position_estimation tgt_kalman_filters ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(target_position_estimation ${catkin_EXPORTED_TARGETS})
###############################################################################

