#! /usr/bin/python3

from airsim.client import MultirotorClient
from team import Team
from target import Target
from math import cos, pi, sin, atan2, pow
import numpy as np

import multiprocessing as mp
import airsim, rospy
from typing import List
import os, sys, json


def getDroneListFromSettings(settingsFilePath: str = None) -> List[str]:
    """
    Loads the list of drones from the airsim setting file

    Args:
        settingsFilePath (str, optional): Path to airsim settings file. Defaults to None.

    Returns:
        List[str]: List of vehicles in file
    """
    if settingsFilePath == None:
        HOME = os.getenv("HOME")
        settings_path = HOME + "/Documents/AirSim/settings.json"
    else:
        settings_path = settingsFilePath

    try:
        settings_file = open(settings_path, "r")
    except Exception:
        print("Error opening settings file. Exiting")
        exit()

    settings = json.loads(settings_file.read())
    settings_file.close()

    vehicle_list = list()
    for i in settings["Vehicles"]:
        vehicle_list.append(i)

    return vehicle_list


def setUpTargets(client: MultirotorClient, target_list: List):
    for t in target_list:
        pose = airsim.Pose(t[2], t[3])
        client.simSetObjectPose(object_name=t[0], pose=pose)


team_list = []


def shutdown() -> None:
    """
    Shuts downs all teams 
    """
    for team in team_list:
        team.shutdown()

    sys.exit(0)


if __name__ == "__main__":

    ######################################
    #
    #     SETUP PYTHON CLIENT
    #

    ip = ""  # UNCOMMENT TO RUN ON LOCALHOST
    #ip = "10.0.0.3"  # "192.168.1.129"         # UNCOMMENT TO RUN ON REMOTE HOST

    client = airsim.MultirotorClient(ip=ip)
    client.confirmConnection()

    lock = mp.Lock()

    ######################################
    #
    #     SET WIND

    wind = airsim.Vector3r(
        0, 0, 0
    )  # CREATE WIND VECTOR -> airsim.Vector3r(n, e, d) [m/s in world frame]
    # client.simSetWind(wind)             # SET WIND

    ######################################
    #
    #     CREATE DRONE/TEAM LISTS

    if ip != "":
        vehicle_list = ["Drone0"]
        # vehicle_list = ["Drone0", "Drone1", "Drone2", "Target0", "Drone3", "Drone4", "Target1"]
    else:
        vehicle_list = getDroneListFromSettings()

    vehicle_list = ["Drone0", "Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6", "Drone7", "Drone8", "Drone9"]
    drone_list = []
    target_list = [
        (
            "African_Poacher_1_WalkwRifleLow_Anim2_2",
            [(1.5, 0, 3), (1.5, pi/10, 2.5), (1.5, 0, 10)],
            airsim.Vector3r(-235, -242, 0),
            airsim.to_quaternion(0, 0, 0),
        ),
        (
            "African_Poacher_1_WalkwRifleLow_Anim3_11",
            [(1.5, 0, 3), (1.5, pi/10, 2.5), (1.5, 0, 10)],
            airsim.Vector3r(-235, -239, 0),
            airsim.to_quaternion(0, 0, 0),
        ),
    ]
    team_list = []
    target_procs = dict()

    for v in vehicle_list:
        if "Drone" in v:
            drone_list.append(v)

    setUpTargets(client, target_list)

    for i in range(len(target_list)):
        target_procs[target_list[i][0]] = Target(
            "Team" + str(i), target_list[i][0], ip=ip, path=target_list[i][1]
        )
        target_procs[target_list[i][0]].start()

        sub_drone = drone_list[
            i
            * len(drone_list)
            // len(target_list) : (i + 1)
            * len(drone_list)
            // len(target_list)
        ]
        s = Team("Team" + str(i), sub_drone, target_procs[target_list[i][0]], client, lock)
        team_list.append(s)

    ######################################
    #
    #     SETUP ROS

    print("SETUP ROS ON PARENT PROCESS")
    rospy.init_node("swarm")
    rospy.on_shutdown(shutdown)

    for i in team_list:
        i.setup_ros()

    ######################################
    #
    #     RUN SIMULATION

    print("TAKING OFF")
    for team in team_list:
        team.takeoff(False)


    rate = rospy.Rate(10)

    t = rospy.get_time()

    while rospy.get_time() - t < 10:
        c = np.zeros((len(team_list), 3))
        for i in range(0, len(team_list)):
            c[i] = team_list[i].calculateCentroid()
            
        print("Centroid 1: " + str(c[0]) + "\tCentroid 2: " + str(c[1]))

    '''
    rospy.sleep(5)

    # print("MOVE TO [0,0,-4]")
    # team_list[0].move_to_location(target=[10,10,-4], timeout=10, tolerance=0.5)
    # team_list[0].wait()
    
    print("BEGIN TRACKING")
    for team in team_list:
        team.track_object(20,-4)


    for team in team_list:
        team.wait()

    print("LANDING")
    for team in team_list:
        team.land(False)

    rospy.sleep(5)

    '''

    print("SHUTDOWN")
    for team in team_list:
        team.shutdown()

    for t in target_list:
        target_procs[t[0]].shutdown()

    print("SIMULATION ENDED")
