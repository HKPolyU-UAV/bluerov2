# BlueROV2 MPC
Designing MPC for bluerov2 model.

## Requirements
* [uuv simulator](https://uuvsimulator.github.io/)
* [bluerov2 ros simulation](https://github.com/tsaoyu/bluerov2)
* [ros kinetic or newer](http://wiki.ros.org/ROS/Installation)
* Python

## Run bluerov2 control
Ocean_waves in uuv simulator is set as default world.

Control with teleop:

```
roslaunch bluerov2_gazebo start_pid_demo_with_teleop.launch
```

Control without teleop:

```
roslaunch bluerov2_gazebo start_pid_demo.launch
```

Linear trajectory:

```
roslaunch uuv_control_utils send_waypoints_file.launch uuv_name:=bluerov2 interpolator:=linear
```

Helical trajectory:

```
roslaunch uuv_control_utils start_helical_trajectory.launch uuv_name:=bluerov2 n_turns:=2
```

Go to desired position by controlling cmd_vel:

```
rosrun bluerov2_control waypoint.py
```
