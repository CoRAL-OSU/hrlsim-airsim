#!/usr/bin/env bash

roslaunch airsim_ros_pkgs airsim_node.launch host:=$2 publish_clock:=true &
rosparam set /use_sim_time true
