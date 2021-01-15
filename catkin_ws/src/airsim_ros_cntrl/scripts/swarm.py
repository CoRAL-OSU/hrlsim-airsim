#! /usr/bin/python

import json
import os
import time
import multiprocessing as mp
import sys
import signal

import math

import airsim
import rospy, actionlib

from geometry_msgs.msg import TwistStamped, PoseStamped
from airsim_ros_pkgs.srv import Takeoff, TakeoffResponse, Land, LandResponse

from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from airsim_ros_cntrl.msg import TrackObjectAction, TrackObjectFeedback, TrackObjectResult, TrackObjectGoal
from airsim_ros_cntrl.msg import MoveToLocationAction, MoveToLocationFeedback, MoveToLocationResult, MoveToLocationGoal


import drone


swarm_list = list()

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
        rospy.on_shutdown(self.shutdown)

        for i in self.vehicle_list:

            prefix = "/" + self.swarm_name + "/" + i

            cmd_vel_topic_name = prefix + "/cmd/vel"
            cmd_pos_topic_name = prefix + "/cmd/pos"

            pubs = dict()
            pubs['cmd_vel'] = rospy.Publisher(cmd_vel_topic_name, TwistStamped, queue_size=10)
            pubs['cmd_pos'] = rospy.Publisher(cmd_pos_topic_name, PoseStamped, queue_size=10)

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


            track_object_action_name = prefix + "/track_object"
            move_to_location_action_name = prefix + "/move_to_location"

            actions = dict()
            actions['track'] = actionlib.SimpleActionClient(track_object_action_name, TrackObjectAction)
            actions['track'].wait_for_server()

            actions['move_to_location'] = actionlib.SimpleActionClient(move_to_location_action_name, MoveToLocationAction)
            actions['move_to_location'].wait_for_server()

            self.drones[i].pubs = pubs
            self.drones[i].srvs = srvs
            self.drones[i].actions = actions

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
                #return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)

        if wait:
            self.wait()

    def land(self, wait=False):
        for i in self.vehicle_list:
            try:
                resp = self.drones[i].srvs['land'](False)
                #return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)

        if wait:
            self.wait()



    def wait(self):
        for i in self.vehicle_list:
            try:
                resp = self.drones[i].srvs['wait'](True)
                #return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)           


    
    def cmd_pos(self, cmd=None, cmd_all=None, wait=False, drone_name=None):

        if cmd_all != None:
            for i in self.vehicle_list:
                self.drones[i].pubs['cmd_pos'].publish(cmd_all)
        
        else:
            self.drones[drone_name].pubs['cmd_pos'].publish(i)
        

        if wait:
            self.wait()

    


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


    def move_to_location(self, target, timeout, tolerance):
         # Move to location in a circle configuration

        l = 2
        
        delta_theta = 2*math.pi / len(self.vehicle_list)
        
        if len(self.vehicle_list) > 1:
            r = l / delta_theta
        else:
            r = 0

        i = 0

        for drone in self.vehicle_list:
            position = []
            position.append(target[0] + r*math.cos(delta_theta*i))
            position.append(target[1] + r*math.sin(delta_theta*i))
            position.append(target[2])

            yaw_frame = "local"
            yaw = 0.0
            
            i += 1    

            goal = MoveToLocationGoal(target=position, timeout=timeout, tolerance=tolerance, yaw_frame=yaw_frame, yaw=yaw)
            self.drones[drone].actions['move_to_location'].send_goal(goal)

        for drone in self.vehicle_list:
            self.drones[drone].actions['move_to_location'].wait_for_result()        


    def track_object(self, object_name, timeout, z_offset):
        # Track object in a circle configuration

        l = 2
        
        delta_theta = 2*math.pi / len(self.vehicle_list)
        
        if len(self.vehicle_list) > 1:
            r = l / delta_theta
        else:
            r = 0

        i = 0

        for drone in self.vehicle_list:
            dx = r*math.cos(delta_theta*i)
            dy = r*math.sin(delta_theta*i)
            dz = z_offset
            
            i += 1

            offset = (dx, dy, dz)
            goal = TrackObjectGoal(object_name=object_name, timeout=timeout, offset=offset)
            self.drones[drone].actions['track'].send_goal(goal)       


        for drone in self.vehicle_list:
            self.drones[drone].actions['track'].wait_for_result()  

    '''
    def hover(self):
        for i in self.drones:
            self.drones[i].hover()

    '''
    def shutdown(self, shutdown=True):
        print("SHUTDOWN SWARM")

        if not rospy.is_shutdown():
            for i in self.vehicle_list:
                try:
                    resp = self.drones[i].srvs['shutdown'](True)
                    #return resp
                except rospy.ServiceException as e:
                    print("Service call failed: %s" %e)       

        for i in self.vehicle_list:
            self.drones[i].process.join()    


def sigint_handler(sig, frame):
    for swarm in swarm_list:
        swarm.shutdown()

    sys.exit(0)




if __name__ == "__main__":
    swarm = Swarm(swarmName="swarm")
    swarm_list.append(swarm)

    #time.sleep(8)

    print("TAKING OFF")
    swarm.takeoff(False)

    time.sleep(5)


    print("MOVING TO [0,0,-4]")
    swarm.move_to_location(target=[0,0,-4], timeout=10, tolerance=0.5)
    time.sleep(5)

    with swarm.lock:
        #object_name = swarm.client.simListSceneObjects("Deer.*")[0]
        object_name = "Stop_Sign_02_8"

    print("TRACKING OBJECT %s" % object_name)
    swarm.track_object(object_name, 15, -4)

    print("RESULT")
    print(swarm.drones["Drone0"].actions["track"].get_result())

    print("MOVING TO [0,0,-1]")
    swarm.move_to_location(target=[0,0,-1], timeout=10, tolerance=0.5)
    time.sleep(5)

    print("LANDING")
    swarm.land(True)

    time.sleep(10)


    print("SHUTDOWN")
    swarm.shutdown()

    sys.exit(0)

    '''
    with swarm.lock:
        deer_list = swarm.client.simListSceneObjects("Deer.*")
        deer = deer_list[-1]
        print(deer_list)


        pose = swarm.client.simGetObjectPose(deer)
        print("Pose of Animal: " + str(pose))

        poseD = swarm.client.simGetObjectPose("Drone0")
        print("Pose of Drone: " + str(poseD))



    print("MOVING TO ANIMAL")

    with swarm.lock:
        drone_pose = swarm.client.simGetObjectPose("Drone0")
    drone_pose.position.z_val = 0

    while airsim.Vector3r.distance_to(drone_pose.position,pose.position) > 1:
        with swarm.lock:
           pose = swarm.client.simGetObjectPose(deer)
        pose.position.z_val = 0

        pos_cmd.header.frame_id = "global"
        pos_cmd.pose.position.x = pose.position.x_val
        pos_cmd.pose.position.y = pose.position.y_val
        pos_cmd.pose.position.z = poseD.position.z_val

        swarm.cmd_pos(cmd_all=pos_cmd, wait=False)

        time.sleep(0.1)

    with swarm.lock:
        pose = swarm.client.simGetObjectPose(deer)
    
    pos_cmd.header.frame_id = "global"
    pos_cmd.pose.position.x = pose.position.x_val
    pos_cmd.pose.position.y = pose.position.y_val
    pos_cmd.pose.position.z = pose.position.z_val - 2

    time.sleep(10)





    with swarm.lock:
        object_list = swarm.client.simListSceneObjects("Animal.*")
        print(list)


        pose = swarm.client.simGetObjectPose(object_list[-1])
        print("Pose of Animal: " + str(pose))

        poseD = swarm.client.simGetObjectPose("Drone0")
        print("Pose of Drone: " + str(poseD))

        pose.position.z_val -= 2

        print("SETTING POSE OF DRONE TO ANIMAL")
        swarm.client.simSetObjectPose("Drone0", pose)

        poseD = swarm.client.simGetObjectPose("Drone0")
        print(poseD)

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
    '''
