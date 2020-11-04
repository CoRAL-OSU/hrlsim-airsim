#! /usr/bin/python

import json
import os
import time

import drone


def makeVelDict(lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
    return((dict([('lx',lx), ('ly',ly), ('lz',lz), ('ax', ax), ('ay',ay), ('az',az)])))


class Swarm:
    def __init__(self, settingsFilePath=None):

        self.vehicle_list = self.getDroneListFromSettings(settingsFilePath)
        self.drones = list()

        for i in self.vehicle_list:
            self.drones.append(drone.Drone(i))

        print("SWARM CREATED WITH %d DRONES" %len(self.drones))


    def getDroneListFromSettings(self, settingsFilePath=None):
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


    def takeoff(self, wait=False):
        print("SWARM TAKEOFF")

        for i in self.drones:
            i.takeoff(wait)

    def land(self, wait=False):
        print("SWARM LAND")

        for i in self.drones:
            i.land(wait)

    def cmd_vel(self, cmd, frame="body"):
        if frame == "body":
            for i in self.drones:
                i.set_velocity_local(cmd)
        elif frame == "world":
            for i in self.drones:
                i.set_velocity_world(cmd)
        else:
            printf("UNRECOGNIZED FRAME")


if __name__ == "__main__":
    swarm = Swarm()

    print("TAKING OFF AND WAITING")
    swarm.takeoff(True)

    print("MOVE FORWARD AT 2 m/s")
    swarm.cmd_vel(makeVelDict(lx=2), frame="body")


    swarm.land(True)