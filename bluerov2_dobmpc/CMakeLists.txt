cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_dobmpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  uuv_gazebo_ros_plugins_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

find_package(
  Eigen3 REQUIRED
)

set(acados_include "/home/fyp/acados/include")
set(acados_lib "/home/fyp/acados/lib")
set(bluerov2_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)
add_message_files(
  FILES
  Reference.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bluerov2_dobmpc
# CATKIN_DEPENDS uuv_gazebo_plugins_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${bluerov2_model}
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
)

link_directories(${bluerov2_model})

add_executable(bluerov2_dobmpc_node 
  src/bluerov2_dobmpc_node.cpp
  src/bluerov2_dobmpc.cpp
  src/bluerov2_interface.cpp)
  
target_link_libraries(bluerov2_dobmpc_node
  ${bluerov2_model}/libacados_ocp_solver_bluerov2.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)
