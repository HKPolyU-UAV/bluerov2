## BlueROV2 MPC
Designing MPC for bluerov2 model.

# Requirements
* [uuv simulator](https://uuvsimulator.github.io/)
* [bluerov2 ros simulation](https://github.com/tsaoyu/bluerov2)
* [ros kinetic or newer](http://wiki.ros.org/ROS/Installation)
* Python

# Working progress
* Fixed camera function in bluerov2_description/urdf/sensors.xacro
* Created new launch file bluerov2_description/launch/upload_bluerov2_v2.launch
* Created new launch file bluerov2_description/launch/upload_bluerov2_v2_default.launch
* Created new launch file bluerov2_gazebo/launch/start_pid_demo_with_teleop.launch
* Created new launch file bluerov2_control/launch/rov_pid_controller.launch
