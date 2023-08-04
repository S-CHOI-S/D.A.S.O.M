################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dasom_toolbox)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  orocos_kdl
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dasom_toolbox
 CATKIN_DEPENDS roscpp rospy std_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

add_library(dasom_toolbox
  src/dasom_toolbox/dasom_workbench.cpp
  #src/dasom_toolbox/dasom_camera.cpp
)
add_dependencies(dasom_toolbox ${catkin_EXPORTED_TARGETS})
target_link_libraries(dasom_toolbox ${catkin_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dasom_toolbox
  # EXPORT dasom_toolbox
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
