# Elisa3PythonController

## Description
This is a python API for the Elisa3-robots (see https://www.gctronic.com/doc/index.php/Elisa-3) build on ROS and the modified Elisa3_npde_cpp package

## Test instruction
### Code structure
- catkin_ws
  - src
        - elisa3_node_cpp
        - Elisa3PythonController
    
### Preperation
- update robot addresses in 'catkin_ws/src/mocap_optitrack/config/mocap.yaml
- update information on robots in 'carkin_ws/src/Elisa3PythonController/src/mapper.json'


### Launch
- from 'catkin_ws' run 'roslaunch elisa3_node_cpp elisa3_swarm.launch'
- from 'catkin_ws/src/elisa3_node_cpp/Elisa3PythonController/src' run main python.py
