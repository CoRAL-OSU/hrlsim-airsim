# Airsim HLQR Simulator
This repository contains an HLQR simulator based on Airsim. The current settings.json file prepares a simulation with 10 agents, 2 targets, and 1 camera drone at 1/10 real time. It should be executed in the Africa environment.


## Build Procedures
-

## Launch Procedures
- roscd airsim_ros_cntrl/scripts
- roslaunch airsim_ros_pkgs airsim_node.launch host:=<ip> publish_clock:=True
- rosparam set /use_sim_time True
- ./run_sim.py 
