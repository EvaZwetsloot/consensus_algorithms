# Consensus Algorithms
Implementation of the ETC, PETC and Phase algorithms that solve the average consensus problem for single-integrator systems

This repository contains the code used for three implementations:
1. Basic simulation - Simulations consisting of solving an initial value problem with single integrator agents
2. Webot simulation - A simulation of the algorithms applied to the Elisa-3 robots in the Webot simulation environment
3. Real implementation - Implementation of the algorithms on real Elisa-3 robots

# How to use
## Basic simulation
Running the basic simulation is quite straight forward. It only involves setting the network parameters and triggering parameters in the main.py file. After which the simulation can be executed.

## Webot simulation
Each algorithm is implemented in its own world, by opening the world the specific algorithm is executed automatically. In the robot controller and supervisor controller, the Bx, By and L matrices need to be adjusted if an other network or formation is required.

## Real implementation
Code structure:
- catkin_ws
  - src
        - elisa3_node_cpp - Elisa3PythonController
        - mocap_optitrack - mocap.yaml
        - elisa3_firmware

First update the firmware of the robots with the hex file that can be found in the firmware folder. You can find instruction for updating the firmware, here: https://www.gctronic.com/doc/index.php/Elisa-3. Once the firmware is updated, one of the three algorithms can be selected by using the physical selector on the robots:

- Selector 14: phase algorithm
- Selector 13: ETC algorithm
- Selector 12: PETC algorithm

The formation and network topology can be adjusted by changing the Bx, By and L matrices in the elisa3_library.c file. Note that the Bx and By matrices for the phase algorithm need to contain the x and y distance in mm to the centre of the formatio, while for the ETC and PETC algorithms the Bx and By matrices contain the inter-robot distance in mm.

When the elisa3-lib.c file is update to allow for a different formation, do the following:

Build the library cd src/pc-side-elisa3-library/linux make
Build the elisa3_node_cpp package cd ~ cd catkin_ws catkin_make
Run the package by using: roslaunch elisa3_node_cpp elisa3_swarm.launch.

For detailed informations about the node refer to the elisa-3 ROS wiki.

In order to use data from the OptiTrack system, you will also need to the python controller:
- cd catkin_ws/src/elisa3_node_cpp/Elisa3PythonController/src python main.py
