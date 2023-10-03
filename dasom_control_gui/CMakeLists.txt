cmake_minimum_required(VERSION 3.0.2)
project(dasom_control_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  cmake_modules
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  omni_msgs
)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_library(QWT_LIBRARIES NAMES qwt-qt5 HINTS /usr/lib)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(dasom_control_gui_SRCS
  src/dasom_control_gui.cpp
  src/dasom_gui.cpp
  src/ratio_layouted_frame.cpp
  src/dasom_frame.cpp
  src/dasom_node.cpp
  src/qcgaugewidget.cpp
)

set(dasom_control_gui_HDRS
  include/dasom_control_gui/dasom_control_gui.h
  include/dasom_control_gui/dasom_gui.h
  include/dasom_control_gui/ratio_layouted_frame.h
  include/dasom_control_gui/dasom_frame.h
  include/dasom_control_gui/dasom_node.h
  include/qcgaugewidget.h
)

set(dasom_control_gui_UIS
  ui/dasom_rqt.ui
  ui/dasom_rqtt.ui
)

set(dasom_control_gui_RESOURCES
  resources/icons.qrc
  resources/resources.qrc
)

set(dasom_control_gui_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${dasom_control_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp std_msgs sensor_msgs geometry_msgs cv_bridge omni_msgs
)

catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(dasom_control_gui_MOCS ${dasom_control_gui_HDRS})
  qt5_add_resources(dasom_control_gui_RCC ${dasom_control_gui_RESOURCES})
else()
  qt4_wrap_cpp(dasom_control_gui_MOCS ${dasom_control_gui_HDRS})
  qt4_add_resources(dasom_control_gui_RCC ${dasom_control_gui_RESOURCES})
endif()

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(dasom_control_gui_UIS_H ${dasom_control_gui_UIS})
else()
  qt4_wrap_ui(dasom_control_gui_UIS_H ${dasom_control_gui_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${dasom_control_gui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} /usr/include/qwt)

add_executable(dasom_app src/dasom_app.cpp ${dasom_control_gui_SRCS} ${dasom_control_gui_MOCS} ${dasom_control_gui_UIS_H} ${dasom_control_gui_RESOURCES})
add_library(${PROJECT_NAME} ${dasom_control_gui_SRCS} ${dasom_control_gui_MOCS} ${dasom_control_gui_UIS_H} ${dasom_control_gui_RESOURCES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${QWT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(dasom_app Qt5::Widgets ${QWT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ui launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/dasom_control_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

