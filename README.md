# ROS ENVIRONMENT (CATKIN WORKSPACE)
Sofware in the loop (SITL) simulations code to implement the designed control system in my master thesis.

# HOW TO USE THE PACKAGES

To start using the already made and available packages, copy the wanted package into your own catkin workspace source folder, usually under catkin_ws/src. 

Open your terminal in the catkin_ws directory and type:

```
$: catkin build
```
Wait until the process is finished. You should get no errors (in red).

You are now ready to import the scripts inside the packages onto your .launch files.

# AVAILABLE PACKAGES

## Control Module

### mpf_control and particles_control packages

The control system to steer the vehicles to the desired positions is the one described in [Félix's master thesis](https://github.com/mg-felix/sitl-simulations/files/10230432/DM_ALF_ENGEL_139928E_Miguel_Felix.pdf).

### particles_msgs package

Adds the message structures to communicate the particles information. 

## CV for Target Data Module

### target_geolocation package

The computer vision alorithm used to obtain the target's velocity and position is the one described in [Alves's master thesis](https://github.com/mg-felix/sitl-simulations/files/10230459/DissertacaoMestrado_ASPAL_PILAV_140667-B_Alves.pdf).

### target_msgs package

Adds the message structures to communicate the target data.

## Data

All the simulation data is saved directly from the controller into the plot/simulation_data path. 
