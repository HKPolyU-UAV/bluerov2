# BlueROV2 MPC
This work implements a NMPC controller for BlueROV2 with ROS.

## Prerequisites
* Python 3.7
* ROS ([ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) recommended)
* [uuv simulator](https://uuvsimulator.github.io/)
* [MAVROS](http://wiki.ros.org/mavros)
* [Acados](https://docs.acados.org/installation/index.html)

## Getting started
Install python 3.7
```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7
```
Install python dependencies
```
python3 -m pip install pip
pip3 install numpy matplotlib scipy future-fstrings casadi>=3.5.1 setuptools
sudo apt-get install python3.7-tk
```
Install Acados
```
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> ..
make install -j4
```
Create a catkin workspace and clone uuv simulator package to catkin src folder (ex. ~/catkin_ws/src)
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
cd src
sudo apt install ros-kinetic-uuv-simulator	(for ros kinetic)
sudo apt install ros-lunar-uuv-simulator	(for ros lunar)
sudo apt install ros-melodic-uuv-simulator	(for ros melodic)
git clone --branch noetic https://github.com/arturmiller/uuv_simulator.git	(for ros noetic)
```
Clone this package to catkin src folder
```
cd ~/catkin_ws/src
git clone https://github.com/HKPolyU-UAV/bluerov2.git
```
Compile
```
cd ~/catkin_ws
catkin_make
```

## Start simulation without controller
Ocean_waves in uuv simulator is set as default world.

Quick start:
```
roslaunch bluerov2_gazebo quick_start.launch
```
or
Start with thruster manager:
```
roslaunch bluerov2_gazebo start_with_thruster_manager.launch
```
The forces and moments can be published to topic /bluerov2/thruster_manager/input.

## Start simulation with PID controller
Control with teleop (using joystick):
```
roslaunch bluerov2_gazebo start_pid_demo_with_teleop.launch
```
or
Control without teleop:
```
roslaunch bluerov2_gazebo start_pid_demo.launch
```

## Trajectory tracking
Start simulation with PID controller first, then

Following linear trajectory:
```
roslaunch uuv_control_utils send_waypoints_file.launch uuv_name:=bluerov2 interpolator:=linear
```
or
Following helical trajectory:
```
roslaunch uuv_control_utils start_helical_trajectory.launch uuv_name:=bluerov2 n_turns:=2
```
