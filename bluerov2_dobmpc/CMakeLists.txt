cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_dobmpc)

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
find_package(gazebo_msgs REQUIRED)
find_package(PCL 1.8.1 REQUIRED)
find_package(OpenCV REQUIRED)

set(acados_include "~/acados/include")
set(acados_lib "~/acados/lib")
set(bluerov2_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

add_message_files(
  FILES
  Reference.msg
  Pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bluerov2_dobmpc
  LIBRARIES bluerov2_ampc
#  CATKIN_DEPENDS uuv_gazebo_plugins_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS gazebo_msgs
  message_generation
  message_runtime
  ros_utilities

)
# include_directories(/usr/include/pcl-1.12/)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bluerov2_model}
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
  ${PCL_CONVERSIONS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ros_utilities
)

link_directories(${bluerov2_model})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

################################################################################################
# bluerov2_dob_node
add_executable(bluerov2_dob_node 
  src/bluerov2_dob_node.cpp
  src/bluerov2_dob.cpp)

target_link_libraries(bluerov2_dob_node
  ${bluerov2_model}/libacados_ocp_solver_bluerov2.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)

add_dependencies(bluerov2_dob_node uuv_gazebo_ros_plugins_msgs_generate_messages_cpp)

################################################################################################
# bluerov2_dob_ctrl_node
add_executable(bluerov2_dob_ctrl_node 
  src/bluerov2_dob_ctrl_node.cpp
  src/bluerov2_dob_ctrl.cpp
  src/ctrller/mpc.cpp
  src/ctrller/pid.cpp
)

target_link_libraries(bluerov2_dob_ctrl_node
  ${bluerov2_model}/libacados_ocp_solver_bluerov2.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)

add_dependencies(bluerov2_dob_ctrl_node 
  uuv_gazebo_ros_plugins_msgs_generate_messages_cpp
  airo_message_generate_messages_cpp
  ros_utilities
)

################################################################################################
# bluerov2_inspection_node
add_executable(bluerov2_inspection_node src/bluerov2_inspection_node.cpp)  

target_link_libraries(bluerov2_inspection_node 
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(bluerov2_inspection_node uuv_gazebo_ros_plugins_msgs_generate_messages_cpp)

################################################################################################
# bluerov2_ampc_node
add_executable(bluerov2_ampc_node 
  src/bluerov2_ampc_node.cpp
  src/bluerov2_ampc.cpp)

target_link_libraries(bluerov2_ampc_node
  ${bluerov2_model}/libacados_ocp_solver_bluerov2.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)

add_dependencies(bluerov2_ampc_node uuv_gazebo_ros_plugins_msgs_generate_messages_cpp)
