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

$: git clone https://github.com/PX4/Firmware.git PX4-Autopilot --recursive

$: cd PX4-Autopilot

$: bash ./Tools/setup/ubuntu.sh
```

Reboot the computer.
```
$: wget https://github.com/ktelegenov/scripts/blob/main/ubuntu_sim_ros_noetic.sh

$: bash ubuntu_sim_ros_noetic.sh
```
Close the terminal and open it again.
```
$: cd src/PX4-Autopilot

$: git submodule update --init --recursive

$: DONT_RUN=1 make px4_sitl_default gazebo

$: source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

$: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo

$: roslaunch px4 multi_uav_mavros_sitl.launch

```

Close the running simulation on your terminal by pressing Ctrl+C.

Go to your .bashrc file in your Home folder (make hidden files visible) and add this lines of code at the end:

```
source /home/:$(user)/src/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash /home/:$(user)/src/PX4-Autopilot /home/:$(user)/src/PX4-Autopilot/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/:$(user)/src/PX4-Autopilot

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/:$(user)/src/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

Save the file.

Ready to go!

# Running your simulations

For every topic presented, several examples are already implemented and predefined. Check the mentioned folders for working examples.

## Packages

It is possible to create your own packages, to introduce a new feature to the flight simulation.

To create your first test package, follow this tutorial in https://docs.px4.io/main/en/ros/mavros_offboard_python.html.

Following similar steps, you can now create your own packages that add your own specific features such as aircraft control or computer vision for target detection algorithms. The scripts that implement those features go inside the /scripts folder inside your package. You can add whatever scripts you wish here.

## Launch files

The launch files are inside the PX4-Autopilot/launch folder. You can add whatever .launch files and run them through the command $: roslaunch px4 _[your-file-name]_.launch.
In this files you can add your own nodes to automatically instantiate when running roslaunch. This nodes are written in Python and can use other Python files and functions, as you intend.

## Worlds

The world files are inside the PX4-Autopilot/Tools/worlds. You can add whatever .world files in here. You can costumize objects to be detected for instance, as well as define how they behave through time.

## Models

The model files are inside the PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models. Add your models here. You can for instance add cameras or other sensors to your UAVs, for instance, as well as design new objects to add to your world.
