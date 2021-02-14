#! /usr/bin/python3

from team import Team
from target import Target

import multiprocessing as mp
import airsim
import os, sys, json, time


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



team_list = []
def sigint_handler(sig, frame):

    for team in team_list:
        team.shutdown()

    sys.exit(0)

if __name__ == "__main__":
    client = airsim.MultirotorClient(ip="192.168.1.96")
    client.confirmConnection()

    lock = mp.Lock()

    remote_computer = True

    if remote_computer:
        vehicle_list = ["Drone0", "Drone1", "Drone2", "Target0"]
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
        target_procs[target_list[i]] = Target("Team"+str(i), target_list[i], client, lock, path=[tuple([100,0,-2])])
        target_procs[target_list[i]].start()

        sub_drone = drone_list[i*len(drone_list)//len(target_list):(i+1)*len(drone_list)//len(target_list)]
        s = Team("Team" + str(i), sub_drone, target_list[i], client, lock)
        team_list.append(s)



    print("TAKING OFF")
    team_list[0].takeoff(False)
    time.sleep(3)


    print("BEGIN TRACKING")
    team_list[0].track_object("Target0", 150, -5)


    #team_list[0].move_to_location(target=[0,0,-4], timeout=10, tolerance=0.5)
    #time.sleep(5)


    #team_list[0].move_to_location(target=[5,5,-8], timeout=10, tolerance=0.5)
    #time.sleep(5)


    print("LANDING")
    team_list[0].land(True)

    time.sleep(10)

    print("SHUTDOWN")
    team_list[0].shutdown()
