# BlueROV2 MPC
Designing MPC for bluerov2 model.

## Requirements
* [ros kinetic or newer](http://wiki.ros.org/ROS/Installation)
* [uuv simulator](https://uuvsimulator.github.io/)
* [mavros](http://wiki.ros.org/mavros)
* Python

## Installation
Clone this repository into ```src``` file of your catkin workspace:

```
cd ~/catkin_ws/src
```

```
git clone https://github.com/yahu3198/bluerov2.git
```

Build catkin workspace again:

```
cd ~/catkin_ws
```

```
catkin_make
```
or
```
catkin build
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
Then the forces and moments can be published to topic /bluerov2/thruster_manager/input.


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
