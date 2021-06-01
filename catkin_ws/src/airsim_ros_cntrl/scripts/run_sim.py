#! /usr/bin/python3

from airsim.client import MultirotorClient
from team import Team
from target import Target
from math import cos, pi, sin, atan2, pow
import numpy as np

from camera import Camera

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
    #vehicle_list = ["Drone0", "Drone1", "Drone2", "Drone3"]

    drone_list = []
    target_list = [
        (
            "African_Poacher_1_WalkwRifleLow_Anim2_2",
            [(0,0,19), (0,0,7), (1.5, 0, 3.5), (1.5, -pi/10, 3), (1.5, -0.01, 10), (1.5,pi/5,1), (1.5, 0, 15), (1.5,-pi/5,1), (1.5,-0.02,30)],
            airsim.Vector3r(-250, -312, 0),
            airsim.to_quaternion(0, 0, 2.32),
        ),
        (
            "African_Poacher_1_WalkwRifleLow_Anim3_11",
            [(0,0,19), (1.5, 0, 8), (1.5, -pi/10, 3.5), (1.5, -0.01, 10), (1.5,-pi/10,1), (1.5,0,15), (1.5,pi/10,1), (1.5,-0.02,30)],
            airsim.Vector3r(-259, -318, 0),
            airsim.to_quaternion(0, 0, 2.32),
        ),
    ]
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
        s = Team("Team" + str(i), sub_drone, target_procs[target_list[i][0]])
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


    rospy.sleep(1)
    camera = Camera("Camera")
    camera.start()

    
    print("TAKING OFF")
    for team in team_list:
        team.takeoff()

    rospy.sleep(5)
    team_list[0].activateAgents(-5)
    team_list[1].activateAgents(-10)

    team_list[0].moveInFormation([-245,-255,-5], 2)
    team_list[1].moveInFormation([-240,-255,-10],2)
    
    #for team in team_list:
    #    #team.takeoff()
    #    team.activateAgents(-5)
    

    rospy.sleep(10)

    #team.moveInFormation([-250,-250,-5], 1.5)
    #rospy.sleep(10)
    
    #team.trackTargetInFormation(20, -10)

    #rospy.sleep(20)

    # print("MOVE TO [0,0,-4]")
    # team_list[0].move_to_location(target=[10,10,-4], timeout=10, tolerance=0.5)
    # team_list[0].wait()


    
    print("BEGIN TRACKING")
    team_list[0].track_object(90, -5)
    team_list[1].track_object(90,-10)
    
    #for team in team_list:
    #    team.track_object(40,-4)
    
    for team in team_list:
        team.wait()
    



    print("LANDING")
    for team in team_list:
        team.land()

    rospy.sleep(5)
    
    print("SHUTDOWN")
    for team in team_list:
        team.shutdown()
    

    for t in target_list:
        target_procs[t[0]].shutdown()

    print("SIMULATION ENDED")
