#! /usr/bin/python
from swarm import Swarm
import os
import json
import time

def getDroneListFromSettings(settingsFilePath=None):
    if settingsFilePath == None:
        HOME = os.getenv('HOME')
        settings_path = HOME + '/Documents/AirSim/settings.json'
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

if __name__ == "__main__":
    vehicle_list = getDroneListFromSettings("/mnt/c/Users/Andrew/Documents/AirSim/settings.json")
    drone_list = []
    target_list = []
    for v in vehicle_list:
        if "Drone" in v:
            drone_list.append(v)
        elif "Target" in v:
            target_list.append(v)

    swarm_list = []
    for i in range(len(target_list)):
        sub_drone = drone_list[i*len(drone_list)//len(target_list):(i+1)*len(drone_list)//len(target_list)]
        s = Swarm("Swarm" + str(i), sub_drone, target_list[i], ip="192.168.1.2")
        swarm_list.append(s)

    swarm_list[0].takeoff(False)
    time.sleep(5)

    swarm_list[0].move_to_location(target=[0,0,-4], timeout=10, tolerance=0.5)
    time.sleep(5)

    print("LANDING")
    swarm_list[0].land(True)

    time.sleep(10)

    print("SHUTDOWN")
    swarm_list[0].shutdown()
