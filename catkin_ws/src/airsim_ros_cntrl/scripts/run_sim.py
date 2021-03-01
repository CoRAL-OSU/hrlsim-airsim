#! /usr/bin/python3

from .team import Team
from .target import Target

import multiprocessing as mp
import airsim, rospy
from typing import List
import os, sys, json, time


def getDroneListFromSettings(settingsFilePath: str=None) -> List[str]:
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


team_list = []

def shutdown() -> None:
    """
    Shuts downs all teams 
    """
    for team in team_list:
        team.shutdown()

    sys.exit(0)


if __name__ == "__main__":
    ip = "192.168.1.2"
    client = airsim.MultirotorClient(ip=ip)
    client.confirmConnection()

    lock = mp.Lock()

    if ip is not None:
        vehicle_list = [
            "Drone0",
            "Drone1",
            "Drone2",
            "Target0",
            "Drone3",
            "Drone4",
            "Target1",
        ]
    else:
        vehicle_list = getDroneListFromSettings()

    drone_list = []
    target_list = []
    target_procs = dict()
    for v in vehicle_list:
        if "Drone" in v:
            drone_list.append(v)
        elif "Target" in v:
            target_list.append(v)

    team_list = []

    for i in range(len(target_list)):
        target_procs[target_list[i]] = Target(
            "Team" + str(i),
            target_list[i],
            client,
            lock,
            path=[tuple([100 + (3 * (-(1 ** i))), 0, -2])],
        )
        target_procs[target_list[i]].start()

        sub_drone = drone_list[
            i
            * len(drone_list)
            // len(target_list) : (i + 1)
            * len(drone_list)
            // len(target_list)
        ]
        s = Team("Team" + str(i), sub_drone, target_list[i], client, lock)
        team_list.append(s)

    rospy.init_node("swarm")
    rospy.on_shutdown(shutdown)

    for i in team_list:
        i.setup_ros()

    print("TAKING OFF")
    team_list[0].takeoff(False)
    team_list[1].takeoff(False)
    time.sleep(5)

    print("BEGIN TRACKING")
    team_list[0].track_object("Target0", 25, -10)
    team_list[1].track_object("Target1", 25, -10)

    for i in team_list:
        i.wait()

    team_list[0].track_object("Target0", 5, 0)
    team_list[1].track_object("Target1", 5, 0)

    for i in team_list:
        i.wait()

    # team_list[0].move_to_location(target=[0,0,-4], timeout=10, tolerance=0.5)
    # time.sleep(5)

    # team_list[0].move_to_location(target=[5,5,-8], timeout=10, tolerance=0.5)
    # time.sleep(5)

    print("LANDING")
    team_list[0].land(False)
    team_list[1].land(False)

    time.sleep(5)

    print("SHUTDOWN")
    team_list[0].shutdown()
    team_list[1].shutdown()
