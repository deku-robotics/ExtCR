cmake_minimum_required(VERSION 2.8.3)
add_definitions(
               -std=c++11
#               "-Wall" # enable all warnings
)
project(actuator_controller)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  dynamic_reconfigure
#  joint_controller
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   actuator_cmd.msg
   actuator_vel_cmd.msg
   actuator_pos_cmd.msg
   actuator_state.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   setInt.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs

 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
CATKIN_DEPENDS
dynamic_reconfigure
message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES limit_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
# add_library(limit_controller_lib /src/limit_controller_node.cpp )

## Declare a cpp executable
# add_executable(actuator_controller src/main_actuator_controller.cpp src/actuator_controller_node.cpp include/actuator_controller_node.h include/actuator_state.h include/actuator.h include/faulhaber.h
#include/com_interface.h include/uart.h src/uart.cpp src/faulhaber.cpp)

 add_executable(actuator_controller
    src/main_actuator_controller.cpp
    src/actuator_controller_node.cpp
    include/actuator_controller_node.h
    include/actuator_state.h
    include/actuator.h
    include/limit_check.h
    src/faulhaber.cpp
    include/faulhaber.h
    include/uart.h
    src/uart.cpp

    src/callbackUart.cpp
    src/limit_check.cpp
    include/BeagleBoneBlack-GPIO/GPIO/GPIOConst.h
    include/BeagleBoneBlack-GPIO/GPIO/GPIOConst.cpp
    include/BeagleBoneBlack-GPIO/GPIO/GPIOManager.h
    include/BeagleBoneBlack-GPIO/GPIO/GPIOManager.cpp

    src/Timer.cpp
    include/Timer.hpp
    )
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(actuator_controller ${PROJECT_NAME}_generate_messages_cpp  ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(actuator_controller
   ${catkin_LIBRARIES}
 )


