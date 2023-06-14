# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rqt_mypkg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irqt_mypkg:/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rqt_mypkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" NAME_WE)
add_custom_target(_rqt_mypkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rqt_mypkg" "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rqt_mypkg
  "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rqt_mypkg
)

### Generating Services

### Generating Module File
_generate_module_cpp(rqt_mypkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rqt_mypkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rqt_mypkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rqt_mypkg_generate_messages rqt_mypkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" NAME_WE)
add_dependencies(rqt_mypkg_generate_messages_cpp _rqt_mypkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rqt_mypkg_gencpp)
add_dependencies(rqt_mypkg_gencpp rqt_mypkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rqt_mypkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rqt_mypkg
  "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rqt_mypkg
)

### Generating Services

### Generating Module File
_generate_module_eus(rqt_mypkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rqt_mypkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rqt_mypkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rqt_mypkg_generate_messages rqt_mypkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" NAME_WE)
add_dependencies(rqt_mypkg_generate_messages_eus _rqt_mypkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rqt_mypkg_geneus)
add_dependencies(rqt_mypkg_geneus rqt_mypkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rqt_mypkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rqt_mypkg
  "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rqt_mypkg
)

### Generating Services

### Generating Module File
_generate_module_lisp(rqt_mypkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rqt_mypkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rqt_mypkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rqt_mypkg_generate_messages rqt_mypkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" NAME_WE)
add_dependencies(rqt_mypkg_generate_messages_lisp _rqt_mypkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rqt_mypkg_genlisp)
add_dependencies(rqt_mypkg_genlisp rqt_mypkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rqt_mypkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rqt_mypkg
  "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rqt_mypkg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rqt_mypkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rqt_mypkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rqt_mypkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rqt_mypkg_generate_messages rqt_mypkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" NAME_WE)
add_dependencies(rqt_mypkg_generate_messages_nodejs _rqt_mypkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rqt_mypkg_gennodejs)
add_dependencies(rqt_mypkg_gennodejs rqt_mypkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rqt_mypkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rqt_mypkg
  "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg
)

### Generating Services

### Generating Module File
_generate_module_py(rqt_mypkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rqt_mypkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rqt_mypkg_generate_messages rqt_mypkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choisol/catkin_ws/src/rqt_mypkg_icros/msg/DasomDynamixel.msg" NAME_WE)
add_dependencies(rqt_mypkg_generate_messages_py _rqt_mypkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rqt_mypkg_genpy)
add_dependencies(rqt_mypkg_genpy rqt_mypkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rqt_mypkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rqt_mypkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rqt_mypkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rqt_mypkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rqt_mypkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rqt_mypkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rqt_mypkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rqt_mypkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rqt_mypkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rqt_mypkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rqt_mypkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rqt_mypkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rqt_mypkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rqt_mypkg
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rqt_mypkg_generate_messages_py std_msgs_generate_messages_py)
endif()
