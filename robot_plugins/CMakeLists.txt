cmake_minimum_required(VERSION 2.8.3)
project(robot_plugins)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  gazebo_ros
  gazebo_plugins
  geometry_msgs
  image_transport
  kobuki_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  hast_gazebo_msgs
  tf
  tf2_ros
)

## System dependencies are found with CMake's conventions
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(
  Boost 
  REQUIRED COMPONENTS 
  thread
)
find_package(gazebo REQUIRED)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBias.srv
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pid_controller.msg
  pid_controllers.msg
  pid3.msg
)
  # pid_controller1.msg
  

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
    ardrone_ros_baro 
    ardrone_simple_controller
    ardrone_state_controller
    # hast_vicon_diff_drive
    # hector_gazebo_ros_baro 
    # hector_gazebo_quadrotor_simple_controller
    # hector_gazebo_quadrotor_state_controller
    reset_plugin 
    # gazebo_vicon_diff_drive
    # gazebo_ros_kobuki
   CATKIN_DEPENDS 
    gazebo_plugins
    geometry_msgs
    kobuki_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    image_transport 
    hast_gazebo_msgs
   DEPENDS 
    # system_lib
    tf
    tf2_ros
)


## Declare a cpp library
add_library(reset_plugin src/reset_plugin.cpp)

## Declare a cpp executable
add_executable(test_trajectory src/test_trajectory.cpp)
  target_link_libraries(test_trajectory ${catkin_LIBRARIES})

add_library(ardrone_ros_baro src/gazebo_ros_baro.cpp)
  add_dependencies(ardrone_ros_baro robot_plugins_generate_messages_cpp)

add_library(ardrone_simple_controller src/ardrone_simple_controller.cpp)
  add_dependencies(ardrone_simple_controller robot_plugins_generate_messages_cpp)

add_library(ardrone_state_controller src/ardrone_state_controller.cpp)
  add_dependencies(ardrone_state_controller robot_plugins_generate_messages_cpp)
  target_link_libraries(ardrone_state_controller ${image_transport_LIBRARIES})

# add_library(gazebo_vicon_diff_drive src/gazebo_vicon_diff_drive.cpp)
#   target_link_libraries(gazebo_vicon_diff_drive gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(ugv2_gazebo_ros_kobuki 
      src/gazebo_ros_kobuki_vicon.cpp 
      src/gazebo_ros_kobuki_vicon_updates.cpp 
      src/gazebo_ros_kobuki_vicon_loads.cpp)
  add_dependencies(ugv2_gazebo_ros_kobuki ${catkin_EXPORTED_TARGETS})
  target_link_libraries(ugv2_gazebo_ros_kobuki ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
