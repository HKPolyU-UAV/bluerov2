cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_states)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin COMPONENTS
  nodelet
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

roslaunch_add_file_check(launch)

add_message_files(
  FILES
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS include

    CATKIN_DEPENDS 
    geometry_msgs 
    mavros_msgs 
    roscpp
    std_msgs
    message_generation
)

set(Sophus_LIBRARIES libSophus.so)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_library(ImuDo
  src/ImuDo.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(ImuDo 
    ${catkin_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)
