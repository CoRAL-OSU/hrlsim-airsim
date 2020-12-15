#! /usr/bin/python

import json
import os
import time
import multiprocessing as mp

import math

import airsim
import rospy

from geometry_msgs.msg import TwistStamped, PoseStamped
from airsim_ros_pkgs.srv import Takeoff, TakeoffResponse, Land, LandResponse

from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu


import drone


class Swarm:
    class DroneInfo:
        name = None
        process = None
        pubs = None 
        subs = None 
        services = None 
        actions = None 

        def __init__(self, DroneName, process, pubs, subs, services, actions):
            self.name       = DroneName
            self.process    = process
            self.pubs       = pubs
            self.subs       = subs
            self.services   = services
            self.actions    = actions


    def __init__(self, swarmName, settingsFilePath=None):       
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        self.swarm_name = swarmName
        self.vehicle_list = self.getDroneListFromSettings(settingsFilePath)
        self.drone_procs = dict()
        self.drone_pubs  = dict()

        self.drones = dict()

        self.lock = mp.Lock()

        for i in self.vehicle_list:
            proc = drone.Drone(self.swarm_name, i, self.client, self.lock)

            self.drones[i] = Swarm.DroneInfo(i, proc, None, None, None, None)
            self.drones[i].process.start()

        rospy.init_node(self.swarm_name)

        for i in self.vehicle_list:

            prefix = "/" + self.swarm_name + "/" + i

            cmd_vel_topic_name = prefix + "/cmd/vel"

            pubs = dict()
            pubs['cmd_vel'] = rospy.Publisher(cmd_vel_topic_name, TwistStamped, queue_size=10)

            takeoff_srv_name = prefix + "/takeoff"
            land_srv_name = prefix + "/land"
            wait_srv_name = prefix + "/wait"
            shutdown_srv_name = prefix + "/shutdown"

            rospy.wait_for_service(takeoff_srv_name)
            rospy.wait_for_service(land_srv_name)
            rospy.wait_for_service(wait_srv_name)
            rospy.wait_for_service(shutdown_srv_name)

            srvs = dict()
            srvs['takeoff'] = rospy.ServiceProxy(takeoff_srv_name, Takeoff)
            srvs['land'] = rospy.ServiceProxy(land_srv_name, Land)
            srvs['wait'] = rospy.ServiceProxy(wait_srv_name, SetBool)
            srvs['shutdown'] = rospy.ServiceProxy(shutdown_srv_name, SetBool)

            self.drones[i].pubs = pubs
            self.drones[i].srvs = srvs

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
        for i in self.vehicle_list:
            try:
                resp = self.drones[i].srvs['takeoff'](False)
            
                print(i + " takeoff resp: %d" %resp.success)
                return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)

        if wait:
            self.wait()

    def land(self, wait=False):
        for i in self.vehicle_list:
            try:
                resp = self.drones[i].srvs['land'](False)
                return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)

        if wait:
            self.wait()



    def wait(self):
        for i in self.vehicle_list:
            try:
                resp = self.drones[i].srvs['wait'](True)
                return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)           


    '''
    def cmd_pos(self, cmd=None, cmd_all=None, wait=False):
        if cmd_all != None:
            for i in self.vehicle_list:
                self.drones[i].set_position(cmd_all)
        
        else:
            for i in cmd:
                self.drones[i["drone"]].set_position(i)

        if wait:
            for i in self.vehicle_list:
                self.drones[i].wait_for_cmd()

    '''


    def cmd_vel(self, cmd=None, cmd_all=None, dur=0, drone_name=None):

        if cmd_all != None:
            for i in self.vehicle_list:
                self.drones[i].pubs['cmd_vel'].publish(cmd_all)
        
        else:
            self.drones[drone_name].pubs['cmd_vel'].publish(i)
        
        time_start = time.time()
        while(time.time() - time_start <= dur):
            if cmd_all != None:
                for i in self.vehicle_list:
                    self.drones[i].pubs['cmd_vel'].publish(cmd_all)

            else:
                self.drones[drone_name].pubs['cmd_vel'].publish(i)

            time.sleep(0.05)


    '''
    def hover(self):
        for i in self.drones:
            self.drones[i].hover()

    '''
    def shutdown(self, shutdown=True):
        print("SHUTDOWN SWARM")
        for i in self.vehicle_list:
            try:
                resp = self.drones[i].srvs['shutdown'](True)
                #return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)       

        for i in self.vehicle_list:
            self.drones[i].process.join()    



if __name__ == "__main__":
    swarm = Swarm(swarmName="swarm")
    cmd_vel_list = list()


    print("TAKING OFF")
    swarm.takeoff(False)

    list = swarm.client.simListSceneObjects()
    print(list)


    pose = swarm.client.simGetObjectPose("Drone5")
    print(pose)

    #time.sleep(5)

    vel_cmd = TwistStamped()
    
    print("CLIMB FOR 5 SECONDS AT 3 m/s")
    swarm.cmd_vel(cmd_all=drone.makeVelCmd(frame="global", lz=-1), dur=5)

    #cmd_vel_list.append(drone.makeVelCmd(lz=-2.1), dur=5, drone_name="Drone0")
    #swarm.cmd_vel(cmd=cmd_vel_list, frame="world")
    #time.sleep(5)


    
    print("MOVE IN A QUARTER CIRCLE WITH RADIUS 2 m AT 3 m/s")
    lin_vel = 0.25
    radius = 2.0
    angular_vel = lin_vel/radius
    angular_change = math.pi/2.0
    time_wait = angular_change/angular_vel
    print("Time wait: %f" %time_wait)
    print("Angular velocity: %f" %angular_vel)

    time_start = time.time()
    swarm.cmd_vel(cmd_all=drone.makeVelCmd(frame="local", lx=lin_vel, az=angular_vel), dur=time_wait)
    #time.sleep(time_wait)
    time_taken = time.time() - time_start
    print("Time taken: %f" %time_taken)
    

    #print("MOVE OUT FOR 5 m AT 2 m/s")
    #swarm.cmd_pos(cmd_all=drone.makePosCmd(frame="body", y=-5, vel=2, timeout=10), wait=True)

    #time.sleep(time_wait)
    
    #print("HOVERING")
    #swarm.hover()
    #time.sleep(3)

    print("DESCENDING")
    swarm.cmd_vel(cmd_all=drone.makeVelCmd(frame="local", lz=2), dur=5)
    #time.sleep(5)


    print("LANDING")
    swarm.land(wait=True)
    #time.sleep(10)


    swarm.shutdown()
