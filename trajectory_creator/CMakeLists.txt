cmake_minimum_required(VERSION 2.8.3)
project(trajectory_creator)

set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pelican_ctrl_pkg
  tgt_pstion
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES trajectory_creator
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  DEPENDS system_lib
)


include_directories(include ../tgt_pstion/include ${catkin_INCLUDE_DIRS})

###############################################################################
add_library(trajectory_pelican src/library/trajectory_pelican.cpp)
target_link_libraries(trajectory_pelican target_position_estimation ${catkin_LIBRARIES})
add_dependencies(trajectory_pelican ${catkin_EXPORTED_TARGETS})
###############################################################################
add_executable(trajectory_pelican_single src/trajectory_pelican_single.cpp)
target_link_libraries(trajectory_pelican_single trajectory_pelican ${catkin_LIBRARIES})
add_dependencies(trajectory_pelican_single ${catkin_EXPORTED_TARGETS})
