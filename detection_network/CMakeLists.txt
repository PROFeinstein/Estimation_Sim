cmake_minimum_required(VERSION 2.8.3)
project(detection_network)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
message_runtime
)


## System dependencies are found with CMake's conventions

# find_package(Boost REQUIRED COMPONENTS system)
set(CODEGEN_FCN_NAME yolov2_detect)
set(CODEGEN_DIR /home/mahmoud/Documents)
set(INTEL_MKLDNN_LIB /usr/local/mkl-dnn/lib)
set(INTEL_MKLDNN /usr/local/mkl-dnn)
set(INTEL_MKLDNN_INCLUDE_DIRS /usr/local/mkl-dnn/include)
###                END                   ###


 add_message_files(DIRECTORY msg 
   FILES
   pixel.msg
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   # Or other packages containing msgs
 )

# This line moves the binary weights and biases to the folder where the node executable is so they can be used at runtime
install(DIRECTORY ${CODEGEN_DIR}/codegen/lib/${CODEGEN_FCN_NAME}/ 
       DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_LOCATION}/)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detection_network
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib

)

# Include directories for building
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CODEGEN_DIR}/codegen/lib/${CODEGEN_FCN_NAME}/
  ${INTEL_MKLDNN_INCLUDE_DIRS}

link_directories(${INTEL_MKLDNN_INCLUDE_DIRS})

)

# Create an executable
add_executable(yolov2_detect src/yolov2_detect.cpp )

# Libraries to link against
target_link_libraries(yolov2_detect ${catkin_LIBRARIES} ${CODEGEN_DIR}/codegen/lib/${CODEGEN_FCN_NAME}/${CODEGEN_FCN_NAME}.a ${CODEGEN_DIR}/codegen/lib/${CODEGEN_FCN_NAME} ${INTEL_MKLDNN_LIB}/libmkldnn.so ${INTEL_MKLDNN_LIB}/libmkldnn.so.0 ${INTEL_MKLDNN_LIB}/libiomp5.so ${INTEL_MKLDNN_LIB}/libmkldnn.so.0.14.0 ${INTEL_MKLDNN_LIB}/libmklml_gnu.so ${INTEL_MKLDNN_LIB}/libmklml_intel.so)

# Add ROS dependencies to the executable
add_dependencies(yolov2_detect sensor_msgs_generate_messages_cpp)
