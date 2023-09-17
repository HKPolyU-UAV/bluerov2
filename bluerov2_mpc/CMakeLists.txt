cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  uuv_gazebo_ros_plugins_msgs
)

find_package(
  Eigen3 REQUIRED
)

set(acados_include "~/acados/include")
set(acados_lib "~/acados/lib")
set(bluerov2_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bluerov2_mpc
CATKIN_DEPENDS uuv_gazebo_plugins_msgs
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

add_executable(bluerov2_mpc_node src/bluerov2_mpc_node.cpp)
target_link_libraries(bluerov2_mpc_node
  ${bluerov2_model}/libacados_ocp_solver_bluerov2.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)
add_dependencies(bluerov2_mpc_node uuv_gazebo_ros_plugins_msgs_generate_messages_cpp)
