# BlueROV2 MPC
This work implements a NMPC controller for BlueROV2 with ROS.

This repository contains the robot description and necessary launch files to
simulate the BlueROV2 (unmanned underwater vehicle) on [Unmanned Underwater Vehicle Simulator (UUV Simulator)](https://github.com/uuvsimulator/uuv_simulator). Additional it's possible run BlueROV2 in SITL using [mavros](http://wiki.ros.org/mavros), joystick interaction and video streaming capture with opencv based on [bluerov_ros_playground](https://github.com/patrickelectric/bluerov_ros_playground) package from BlueRobotics.

This work consolidates the contributions from [Ingeniarius, Lda.](http://ingeniarius.pt/) and [Instituite of Systems and Robotics University of Coimbra](https://www.isr.uc.pt/) within the scope of MS thesis "Localization of an unmanned underwater vehicle using multiple water surface robots, multilateration, and sensor data fusion".


## Prerequisites
* Python 3.7
* ROS ([ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) recommended)
* [uuv simulator](https://uuvsimulator.github.io/)
* [MAVROS](http://wiki.ros.org/mavros)
* [Acados](https://docs.acados.org/installation/index.html)

We also provide its [docker image](https://github.com/HKPolyU-UAV/airo_docker_lib) to save you some time.


## Getting started
1. Install python 3.7 and its dependencies
    ```
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt update
    sudo apt install -y python3.7

    apt-get install -y python3-pip

    pip3 install numpy matplotlib scipy future-fstrings casadi>=3.5.1 setuptools
    sudo apt-get install -y python3.7-tk
    ```
2. Install Acados
    ```
    git clone https://github.com/acados/acados.git
    cd acados
    git submodule update --recursive --init
    mkdir -p build
    cd build
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> ..
    make install -j4

    pip install -e ~/acados/interfaces/acados_template

    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/root/acados/lib"' >> ~/.bashrc 
    echo 'export ACADOS_SOURCE_DIR="/root/acados"' >> ~/.bashrc
    source ~/.bashrc
    ```

3. Install ros uuv-simulator packages
    ```
    sudo apt-get install -y ros-noetic-geodesy
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone --branch noetic https://github.com/arturmiller/uuv_simulator.git
    ```

4. Install then compile this package
    ```
    cd ~/catkin_ws/src && \
    git clone --branch huyang-backup https://github.com/HKPolyU-UAV/bluerov2.git
    ```
    Please edit CMakelists.txt [here](/bluerov2_dobmpc/CMakeLists.txt) and [here](/bluerov2_mpc/CMakeLists.txt) if you put your acados in otherwise directory. Modify the line ```set(acados_include "~/acados/include")``` and ```set(acados_lib "~/acados/lib")```
    
    Then do
    ```
    cd ~/catkin_ws/src/bluerov2/bluerov2_dobmpc/scripts && \
    yes | python3 generate_c_code.py
    ```
    and
    ```
    cd ~/catkin_ws/src/bluerov2/bluerov2_mpc/scripts && \
    yes | python3 generate_c_code.py
    ```
    The last commands basically run generate_c_code.py at [here](/bluerov2_dobmpc/scripts/) and [here](/bluerov2_mpc/scripts/).

    Finally, do
    ```
    cd ~/catkin_ws && catkin_make
    ```

## Start simulation without controller
Ocean_waves in uuv simulator is set as default world.

Quick start to initialize the Gazebo world and add blueROV2 vehicle
```
roslaunch bluerov2_gazebo quick_start.launch
```
Or start with thruster manager
```
roslaunch bluerov2_gazebo start_with_thruster_manager.launch
```
With thruster manager, forces and moments can be published to topic /bluerov2/thruster_manager/input.

## Start simulation with MPC controller
A demonstration of starting MPC controler to follow circular trajectory can be launched by
```
roslaunch bluerov2_gazebo start_mpc_demo.launch
```
The bluerov2_mpc package also includes more trajectory following tasks such as following leminscate path and self defined waypoints.

## Start simulation with disturbance observer-based MPC controller
A demonstration of starting MPC controler to follow circular trajectory can be launched by
```
roslaunch bluerov2_dobmpc start_dobmpc_demo.launch 
```
The generated disturbances can be adjusted at [here](/bluerov2_dobmpc/launch/start_dobmpc_demo.launch) or [here](/bluerov2_dobmpc/src/bluerov2_dob.cpp).

## Citation
This repository contains the research code for our paper: https://doi.org/10.3390/jmse12010094.

