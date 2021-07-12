import os, json, airsim
from math import pi

from team import Team
from target import Target

from typing import List


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


def setUpTargets(client: airsim.MultirotorClient, target_list: List):
    for t in target_list:
        pose = airsim.Pose(t[2], t[3])
        client.simSetObjectPose(object_name=t[0], pose=pose)


def createDroneTeamLists(client, ip, setupTargets=True):
    if ip != "":
        vehicle_list = ["Drone0"]
        # vehicle_list = ["Drone0", "Drone1", "Drone2", "Target0", "Drone3", "Drone4", "Target1"]
    else:
        vehicle_list = getDroneListFromSettings()

    #vehicle_list = ["Drone0", "Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6", "Drone7", "Drone8", "Drone9"]
    #vehicle_list = ["Drone0", "Drone1", "Drone2", "Drone3"]

    drone_list = []
    team_list = []
    target_list = []
    target_procs = []
    

    for v in vehicle_list:
        if "Drone" in v:
            drone_list.append(v)

    if setupTargets:
        target_procs = dict()
        target_list = [
            (
                "African_Poacher_1_WalkwRifleLow_Anim2_2",
                [(0,0,19), (0,0,7), (1.5, 0, 3.5), (1.5, -pi/10, 3), (1.5, -0.01, 10), (1.5,pi/5,1), (1.5, 0, 15), (1.5,-pi/5,1), (1.5,-0.02,30)],
                airsim.Vector3r(-250, -312, 0),
                airsim.to_quaternion(0, 0, 2.32),
            ),
        ]
        if(len(drone_list) > 1):    
            target_list.append([
                "African_Poacher_1_WalkwRifleLow_Anim3_11",
                [(0,0,19), (1.5, 0, 8), (1.5, -pi/10, 3.5), (1.5, -0.01, 10), (1.5,-pi/10,1), (1.5,0,15), (1.5,pi/10,1), (1.5,-0.02,30)],
                airsim.Vector3r(-259, -318, 0),
                airsim.to_quaternion(0, 0, 2.32),
            ]
        )
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
    
    else:
        team_list.append(Team("Team0", drone_list, None))

    return team_list, drone_list, target_list, target_procs
