#! /usr/bin/python

import json
import os
import time
import threading
import math

#import rospy

import drone


def makePosCmd(drone=None, timeout=3e38, frame="world", x=0, y=0, z=0, vel=0):
    return((dict([('drone',drone), ('frame',frame), ('timeout',timeout), ('x',x), ('y',y), ('z',z), ('vel',vel)])))

def makeVelDict(drone=None, dur=1, lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
    return((dict([('drone',drone), ('dur',dur), ('lx',lx), ('ly',ly), ('lz',lz), ('ax', ax), ('ay',ay), ('az',az)])))


class Swarm:
    def __init__(self, swarmName, settingsFilePath=None):
        #rospy.init_node(swarmName)
        
        self.swarm_name = swarmName
        self.vehicle_list = self.getDroneListFromSettings(settingsFilePath)
        self.drones = dict()
        self.drone_threads = dict()

        for i in self.vehicle_list:
            self.drones[i] = drone.Drone(self.swarm_name, i)
            #self.drones.append(drone.Drone(self.swarm_name, i))
            drone_thread = threading.Thread(target=self.drones[i].fly, args=(100,))
            drone_thread.start()
            self.drone_threads[i] = drone_thread
            #self.drone_threads.append(drone_thread)

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

    def getDroneList(self):
        return self.drones


    def takeoff(self, wait=False):
        for i in self.drones:
            self.drones[i].takeoff(wait)

    def land(self, wait=False):
        for i in self.drones:
            self.drones[i].land(wait)

    def land_all(self, wait=False):
        land = list()

        for i in self.drones:
            land.append(self.drones[i].get_client().landAsync(vehicle_name=i.get_name()))
        
        if wait:
            for i in range(0, len(land)):
                land[i].join()

    def cmd_vel(self, cmd=None, cmd_all=None, frame="body"):
        if cmd_all != None:
            if frame == "body":
                for i in self.drones:
                    self.drones[i].set_velocity_local(cmd_all)
            elif frame == "world":
                for i in self.drones:
                    self.drones[i].set_velocity_world(cmd_all)
            else:
                print("UNRECOGNIZED FRAME")
        else:
            if frame == "body":
                for i in cmd:
                    self.drones[i['drone']].set_velocity_local(i)
            elif frame == "world":
                for i in cmd:
                    self.drones[i['drone']].set_velocity_world(i)
            else:
                print("UNRECOGNIZED FRAME")


    def hover(self):
        for i in self.drones:
            self.drones[i].hover()

    def shutdown(self, shutdown=True):
        print("SHUTDOWN SWARM")
        for i in self.drones:
            self.drones[i].shutdown(shutdown=shutdown)

        #for i in self.drone_threads:
        #    self.drone_threads[i].join()


if __name__ == "__main__":
    swarm = Swarm(swarmName="swarm")
    cmd_vel_list = list()


    print("TAKING OFF")
    swarm.takeoff(False)
    time.sleep(3)

    print("CLIMB FOR 5 SECONDS AT 3 m/s")
    swarm.cmd_vel(cmd_all=makeVelDict(dur=5, lz=-2), frame="world")

    #cmd_vel_list.append(makeVelDict(drone="Drone0", dur=5, lz=-2.1))
    #swarm.cmd_vel(cmd=cmd_vel_list, frame="world")
    time.sleep(5)

    print("MOVE IN A CIRCLE WITH RADIUS 2 m AT 3 m/s")
    lin_vel = 0.5
    radius = 2.0
    angular_vel = lin_vel/radius
    angular_change = 2.0*math.pi
    time_wait = angular_change/angular_vel
    print("Time wait: %f" %time_wait)
    print("Angular velocity: %f" %angular_vel)

    swarm.cmd_vel(cmd_all=makeVelDict(dur=time_wait, lx=lin_vel, az=angular_vel), frame="body")
    time.sleep(time_wait)


    print("DESCENDING")
    swarm.cmd_vel(cmd_all=makeVelDict(lz=2, dur=5), frame="body")
    time.sleep(5)
    
    print("HOVERING")
    swarm.cmd_vel(cmd_all=makeVelDict(), frame="body")
    swarm.hover()


    print("LANDING")
    swarm.land_all(True)
    #time.sleep(10)


    swarm.shutdown()
