cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_path)

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
  ros_utilities
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
#   Reference.msg
#   Pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS 
#   gazebo_msgs
  message_generation
  message_runtime
  ros_utilities
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ros_utilities
)

################################################################################################
# bluerov2_path_node
add_executable(bluerov2_path_node
  src/bluerov2_path_node.cpp
  src/bluerov2_path.cpp)

target_link_libraries(bluerov2_path_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(bluerov2_path_node 
  uuv_gazebo_ros_plugins_msgs_generate_messages_cpp
  airo_message_generate_messages_cpp
  ros_utilities
)

################################################################################################
# bluerov2_path_wrench
add_executable(bluerov2_wrench
  src/bluerov2_wrench.cpp)

target_link_libraries(bluerov2_wrench
  ${catkin_LIBRARIES}
)