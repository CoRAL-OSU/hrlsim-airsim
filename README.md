# Airsim HLQR Simulator
This repository contains an HLQR simulator based on Airsim. The current [settings.json](/settings.json) file prepares a simulation with 10 agents, 2 targets, and 1 camera drone at 1/10 real time. It should be executed in the Africa environment.

## Current Structure
The `airsim_ros_cntrl` package is structured in the following way
- `Drone.py`: A Basic Drone in the airsim simulator
- `Agent.py`: An agent drone as apart of a team
- `Team.py`: Defines a team of drones
- `Target.py`: A simulated target for the agents to track
- `Run_Sim.py`: Sets up teams and starts tracking
- `Camera.py`: Path controlled camera drone 

## Build Procedures
AirsimRos Depends on the following dependencies
- ROS Melodic(Python3)/Noetic
- Python: pymap3d, pycallgraph, control, numpy, scipy, matplotlib
- In addition, the airsim python client needs to be installed by navigating to `./Airsim/PythonClient` and running `pip3 install .`

To build run `catkin_make` in the `catkin_ws`

## Launch Procedures
```sh
roscd airsim_ros_cntrl/scripts
roslaunch airsim_ros_pkgs airsim_node.launch host:=<ip> publish_clock:=True
rosparam set /use_sim_time True
./run_sim.py 
```
