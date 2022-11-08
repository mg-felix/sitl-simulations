# sitl-simulations
Sofware in the loop (SITL) simulations and implementation for my master thesis, using Gazebo, PX4, ROS and QGC.

## Requirements

This tutorial assumes that you have an Ubuntu 20.04 machine set up. If you do not, here are some instructions on how to do it via virtual box: https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview

## QGroundControl

https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html follow the instructions under Ubuntu Linux section. Before running the roslaunch command, open QGC.

# How to start

## Toolchain instalation based on (https://www.youtube.com/watch?v=9Mb-aV3lmZ0):

Follow the steps:
```
$: sudo apt update

$: sudo apt upgrade

$: sudo apt install git # Skip this step if you already have git installed!

$: mkdir src

$: cd src

$: git clone https://github.com/PX4/Firmware.git --recursive

$: cd Firmware

$: bash ./Tools/setup/ubuntu.sh
```

Reboot the computer.
```
$: wget https://raw.githubusercontent.com/ktelegenov/sim_ros_setup_noetic/main/ubuntu_sim_ros_noetic.sh

$: bash ubuntu_sim_ros_noetic.sh
```
Close the terminal and open it again.
```
$: cd src/Firmware

$: git submodule update --init --recursive

$: DONT_RUN=1 make px4_sitl_default gazebo

$: source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

$: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo

$: roslaunch px4 multi_uav_mavros_sitl.launch

```

Close the running simulation on your terminal by pressing Ctrl+C.

Go to your .bashrc file in your Home folder (make hidden files visible) and add this two lines of code at the end:

```
source /home/ciafa/src/Firmware/Tools/simulation/gazebo/setup_gazebo.bash /home/ciafa/src/Firmware /home/ciafa/src/Firmware/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ciafa/src/Firmware

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ciafa/src/Firmware/Tools/simulation/gazebo/sitl_gazebo

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
```

Save the file.

## Include this repository (based on https://docs.px4.io/main/en/ros/mavros_offboard_python.html):

Delete your /src folder inside the catkin_ws workspace folder and follow the steps:
```
$: roscd  # Moves into ~/catkin_ws/devel directory

$: cd .. 

$: git clone https://github.com/mg-felix/sitl-simulations.git src

$: catkin build
```
Ready to go!

# Running your simulations

For every topic presented, several examples are already implemented and predefined. Check the mentioned folders for working examples.

## Scripts

The scripts go inside the catkin_ws/src/offboard_py/scripts folder. You can add whatever scripts you wish here.

## Launch files

The launch files are inside the Firmware/launch folder. You can add whatever .launch files and run them through the command $: roslaunch px4 [your-file-name].launch.
In this files you can add your own nodes to automatically instantiate when running roslaunch. This nodes are written in Python and can use other Python files and functions, as you intend.

## Worlds

The world files are inside the Firmware/Tools/worlds. You can add whatever .world files in here.

## Models

The model files are inside the Firmware/[your-Tools-path-previously-searched]/models. Add your models here.
