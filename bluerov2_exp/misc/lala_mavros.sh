source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=192.168.2.2
export ROS_MASTER_URI=http://192.168.2.2:11311
export ROS_IP=192.168.2.2
roslaunch mavros apm.launch fcu_url:="tcp://0.0.0.0:5777@"