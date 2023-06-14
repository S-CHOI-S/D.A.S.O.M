cmake_minimum_required(VERSION 3.0.2)
project(two_link)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamixel_workbench_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
add_message_files(
  FILES
  DasomDynamixel.msg
)

add_service_files(
  FILES
  param.srv
)
################################################
## Declare ROS messages, services and actions ##
################################################
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES turtles
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs dynamixel_workbench_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Admittance_twoLink.cpp src/Admittance_twoLink.cpp)
add_dependencies(Admittance_twoLink.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Admittance_twoLink.cpp ${catkin_LIBRARIES})

add_executable(velocity_controller.cpp src/velocity_controller.cpp)
add_dependencies(velocity_controller.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_controller.cpp ${catkin_LIBRARIES})

add_executable(velocity_tracking.cpp src/velocity_tracking.cpp)
add_dependencies(velocity_tracking.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_tracking.cpp ${catkin_LIBRARIES})

add_executable(service_call.cpp src/service_call.cpp)
add_dependencies(service_call.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_call.cpp ${catkin_LIBRARIES})

add_executable(admittance_Twolink.cpp src/admittance_Twolink.cpp)
add_dependencies(admittance_Twolink.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_Twolink.cpp ${catkin_LIBRARIES})
