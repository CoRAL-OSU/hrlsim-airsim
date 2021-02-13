#! /usr/bin/python

import time
import multiprocessing as mp
import sys

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


team_list = list()

class Team:
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


    def __init__(self, teamName, vehicle_list, target, ip=""):       
        self.client = airsim.MultirotorClient(ip)
        self.client.confirmConnection()

        self.team_name = teamName
        self.vehicle_list = vehicle_list
        self.drone_procs = dict()
        self.drone_pubs  = dict()

        self.drones = dict()

        self.lock = mp.Lock()

        for i in self.vehicle_list:
            proc = drone.Drone(self.team_name, i, self.client, self.lock)

            self.drones[i] = Team.DroneInfo(i, proc, None, None, None, None)
            self.drones[i].process.start()

        rospy.init_node(self.team_name)
        rospy.on_shutdown(self.shutdown)

        for i in self.vehicle_list:

            prefix = "/" + self.team_name + "/" + i

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
    for team in team_list:
        team.shutdown()

    sys.exit(0)




if __name__ == "__main__":
    team = Team(teamName="team")
    team_list.append(team)

    #time.sleep(8)

    print("TAKING OFF")
    team.takeoff(False)

    time.sleep(5)


    print("MOVING TO [3,3,-30]")
    team.move_to_location(target=[3,3,-30], timeout=15, tolerance=0.5)
    time.sleep(5)

    with team.lock:
        object_name = team.client.simListSceneObjects("Deer.*")[-1]
        #object_name = "Stop_Sign_02_8"

    print("TRACKING OBJECT %s" % object_name)
    team.track_object(object_name, 150, -30)

    print("RESULT")
    print(team.drones["Drone0"].actions["track"].get_result())

    print("MOVING TO [4,4,-5]")
    team.move_to_location(target=[4,4,-5], timeout=10, tolerance=0.5)
    time.sleep(5)

    print("MOVING TO [-1,-1,-5]")
    team.move_to_location(target=[-1,-1,-5], timeout=10, tolerance=0.5)
    time.sleep(5)

    print("MOVING TO [0,0,-1")
    team.move_to_location(target=[0,0,-1], timeout=10, tolerance=0.5)

    print("LANDING")
    team.land(True)

    time.sleep(10)


    print("SHUTDOWN")
    team.shutdown()

    sys.exit(0)
