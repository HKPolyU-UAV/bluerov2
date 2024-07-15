cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_implement)

## Compile as C++11, supported in ROS Kinetic and newer
if (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "7.6") # Assuming GCC 4.9 supports C++14
    add_compile_options(-std=c++11)
else()
    add_compile_options(-std=c++14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  roslaunch
  uuv_gazebo_ros_plugins_msgs
  nav_msgs
  geometry_msgs
  message_generation
  tf
  std_msgs
  gazebo_msgs
  genmsg
  sensor_msgs
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(PCL 1.8.1 REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  )


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES bluerov2_implement
#  CATKIN_DEPENDS uuv_gazebo_plugins_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS gazebo_msgs
  CATKIN_DEPENDS bluerov2_dobmpc
  message_generation
  message_runtime

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/ahrs_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)