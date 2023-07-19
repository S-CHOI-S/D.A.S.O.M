cmake_minimum_required(VERSION 3.0.2)
project(dasom_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamixel_workbench_msgs
  message_generation
  orocos_kdl
  dasom_toolbox
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
add_message_files(
  FILES
  Torqbian.msg
  DasomDynamixel.msg
)

add_service_files(
  FILES
  param.srv
  movingFlag.srv
  admittanceTest.srv
)
################################################
## Declare ROS messages, services and actions ##
################################################
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtles
  CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_msgs dasom_toolbox
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #include
)

################################################################################
# Install
################################################################################
#install(TARGETS dasom_controllers
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(Admittance_TwoLink src/Admittance_TwoLink.cpp)  # 수정
add_dependencies(Admittance_TwoLink ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Admittance_TwoLink ${catkin_LIBRARIES})

add_executable(torque_jacobian src/torque_jacobian.cpp)
add_dependencies(torque_jacobian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(torque_jacobian dasom_toolbox ${catkin_LIBRARIES})

