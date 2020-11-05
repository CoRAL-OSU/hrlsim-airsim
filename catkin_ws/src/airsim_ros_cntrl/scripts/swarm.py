#! /usr/bin/python

import json
import os
import time
import threading

import rospy

import drone


def makeVelDict(lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
    return((dict([('lx',lx), ('ly',ly), ('lz',lz), ('ax', ax), ('ay',ay), ('az',az)])))


class Swarm:
    def __init__(self, swarmName, settingsFilePath=None):
        rospy.init_node(swarmName)
        
        self.swarm_name = swarmName
        self.vehicle_list = self.getDroneListFromSettings(settingsFilePath)
        self.drones = list()
        self.drone_threads = list()

        for i in self.vehicle_list:
            self.drones.append(drone.Drone(self.swarm_name, i))
            drone_thread = threading.Thread(target=self.drones[-1].fly, args=(1000,))
            drone_thread.start()
            self.drone_threads.append(drone_thread)

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
        for i in self.drones:
            i.takeoff(wait)

    def land(self, wait=False):
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
            print("UNRECOGNIZED FRAME")

    def hover(self):
        for i in self.drones:
            i.hover()

    def shutdown(self, shutdown=True):
        print("SHUTDOWN SWARM")
        for i in self.drones:
            i.shutdown(shutdown=shutdown)

        for i in self.drone_threads:
            i.join()


if __name__ == "__main__":
    swarm = Swarm(swarmName="swarm")

    print("TAKING OFF")
    swarm.takeoff(True)
    #time.sleep(3)

    print("CLIMB FOR 5 SECONDS AT 3 m/s")
    swarm.cmd_vel(makeVelDict(lz=-3), frame="world")
    time.sleep(5)

    print("MOVE IN A CIRCLE WITH RADIUS 5 m AT 3 m/s")
    lin_vel = 3.0
    radius = 5.0

    swarm.cmd_vel(makeVelDict(lx=lin_vel, az=lin_vel/radius), frame="body")
    time.sleep(30)

    swarm.cmd_vel(makeVelDict(lz=3), frame="world")
    time.sleep(5)
    
    swarm.hover()


    print("LANDING")
    swarm.land(True)

    swarm.shutdown()
