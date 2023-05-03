# Install script for directory: /home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/choisol/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/msg" TYPE FILE FILES
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/AX.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/EX.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MX.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MX2.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MX2Ext.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MXExt.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/PRO.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/PROExt.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/RX.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XH.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XL.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XL320.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XM.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XMExt.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelState.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelStateList.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelInfo.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelLoadInfo.msg"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DasomDynamixel.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/srv" TYPE FILE FILES
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/JointCommand.srv"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/WheelCommand.srv"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/GetDynamixelInfo.srv"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/DynamixelCommand.srv"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/EECommand.srv"
    "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/test.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/cmake" TYPE FILE FILES "/home/choisol/catkin_ws/build/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/choisol/catkin_ws/devel/include/dynamixel_workbench_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/choisol/catkin_ws/devel/share/roseus/ros/dynamixel_workbench_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/choisol/catkin_ws/devel/share/common-lisp/ros/dynamixel_workbench_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/choisol/catkin_ws/devel/share/gennodejs/ros/dynamixel_workbench_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/choisol/catkin_ws/devel/lib/python2.7/dist-packages/dynamixel_workbench_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/choisol/catkin_ws/devel/lib/python2.7/dist-packages/dynamixel_workbench_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/choisol/catkin_ws/build/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/cmake" TYPE FILE FILES "/home/choisol/catkin_ws/build/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/cmake" TYPE FILE FILES
    "/home/choisol/catkin_ws/build/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgsConfig.cmake"
    "/home/choisol/catkin_ws/build/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs" TYPE FILE FILES "/home/choisol/catkin_ws/src/dynamixel-workbench-msgs/dynamixel_workbench_msgs/package.xml")
endif()

