cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_exp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  rospy
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS rospy geometry_msgs mavros_msgs roscpp std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

# catkin_install_python(PROGRAMS 
#   scripts/aae.py
#   # scripts/mainserver.py
#   scripts/start.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


add_executable(offb
    src/offb.cpp
)

target_link_libraries(offb
   ${catkin_LIBRARIES}
)

