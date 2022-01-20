#! /usr/bin/python3

from multiprocessing import Lock

import math
import numpy as np

import rospy, actionlib

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, Vector3
from airsim_ros_cntrl.msg import Multirotor
from airsim_ros_pkgs.srv import Takeoff, Land
from actionlib_msgs.msg import GoalStatus

from std_srvs.srv import SetBool

from airsim_ros_cntrl.msg import (
    TrackObjectAction,
    TrackObjectGoal,
)
from airsim_ros_cntrl.msg import (
    MoveToLocationAction,
    MoveToLocationGoal,
)

from typing import List, Dict
from target import Target

from drone import DroneInfo
from agent import Agent
from minimum_snap import MinimumSnap


class Team:
    """
    Class to define a team of drones.
    A team is comprised of any number of agents and up to one target to track.
    Manages the deployment and management of all drones.
    """

    def __init__(
        self,
        teamName: str,
        vehicle_list: List[str],
        target: Target
    ) -> None:
        """
        Contructs a team object.
        A team is comprised of any number of agents and up to one target to track. 

        Args:
            teamName (str): The name of the team
            vehicle_list (List[str]): List of agent names
            target (Target): The target to track
        """
        self.team_name = teamName
        self.vehicle_list = vehicle_list
        self.centroid_timer = None
        self.goal = None

        self.drones: Dict[str, Agent] = dict()

        if target != None:
            self.target = DroneInfo(target.drone_name, target)
        else:
            self.target = None

        self.__shutdown = False

        for i in self.vehicle_list:
            proc = Agent(self.team_name, i)

            self.drones[i] = DroneInfo(i, proc)
            self.drones[i].process.start()

        print("SWARM CREATED WITH %d DRONES" % len(self.drones))

    def setup_ros(self) -> None:
        """
        Function to set up ros topics to talk to teams.
        The team itself does not have a node associated with it.
        """

        self.centroid_des_pub = rospy.Publisher(
            "/"+self.team_name+"/centroid_des/multirotor", Multirotor, queue_size=2
        )
        self.centroid_act_pub = rospy.Publisher(
            "/"+self.team_name+"/centroid_act/multirotor", Multirotor, queue_size=2
        )
        if self.target != None:
            target_prefix = "/" + self.team_name + "/" + self.target.name

            srvs = dict()
            srvs["shutdown"] = rospy.ServiceProxy(
                target_prefix + "/shutdown", SetBool
            )
            self.target.services = srvs

            subs = dict()
            subs["multirotor"] = rospy.Subscriber(target_prefix+"/multirotor", Multirotor, self.target_cb)
            self.target.subs = subs

        for i in self.vehicle_list:

            prefix = self.team_name + "/" + i

            pubs = dict()

            takeoff_srv_name = prefix + "/takeoff"
            land_srv_name = prefix + "/land"
            shutdown_srv_name = prefix + "/shutdown"

            rospy.wait_for_service(takeoff_srv_name)
            rospy.wait_for_service(land_srv_name)
            rospy.wait_for_service(shutdown_srv_name)

            srvs = dict()
            srvs["takeoff"] = rospy.ServiceProxy(takeoff_srv_name, Takeoff)
            srvs["land"] = rospy.ServiceProxy(land_srv_name, Land)
            srvs["shutdown"] = rospy.ServiceProxy(shutdown_srv_name, SetBool)

            track_object_action_name = prefix + "/track_object"
            move_to_location_action_name = prefix + "/move_to_location"

            actions = dict()
            actions["track"] = actionlib.SimpleActionClient(
                track_object_action_name, TrackObjectAction
            )
            actions["track"].wait_for_server()

            actions["move_to_location"] = actionlib.SimpleActionClient(
                move_to_location_action_name, MoveToLocationAction
            )
            actions["move_to_location"].wait_for_server()

            subs = dict()
            subs["multirotor"] = rospy.Subscriber(prefix+"/multirotor", Multirotor, self.agent_cb, i)

            self.drones[i].pubs = pubs
            self.drones[i].services = srvs
            self.drones[i].actions = actions
            self.drones[i].subs = subs

    def agent_cb(self, msg: Multirotor, drone_name: str):
        self.drones[drone_name].state = msg

    def target_cb(self, msg: Multirotor):
        self.target.state = msg

                
    def calculateCentroid(self) -> Multirotor:
        centroidPos = np.zeros(3)
        centroidVel = np.zeros(3)
        for drone in self.drones.values():
            pos = drone.state.state.pose.position
            vel = drone.state.state.vel.linear
            centroidPos += np.array([pos.x, pos.y, pos.z])
            centroidVel += np.array([vel.x, vel.y, vel.z])
            

        centroidPos /= len(self.drones.values()) 
        centroidVel /= len(self.drones.values())

        centroid = Multirotor()
        centroid.state.pose.position = Point(*centroidPos)
        centroid.state.vel.linear = Vector3(*centroidVel)
        return centroid

    def activateAgents(self, altitude=-5):
        c = self.calculateCentroid().state.pose.position
        curr_centroid = np.array([[c.x, c.y, c.z]]).T
        ic = np.zeros((3,3))
        fc = np.zeros((3,3))

        goal = curr_centroid + np.array([[0,0,altitude]]).T
        waypoints = np.concatenate((curr_centroid, goal), 1)
        avg_spd = 1.0

        self.traj = MinimumSnap(waypoints, ic, fc, avg_spd)
        self.t0 = rospy.get_time()

        self.centroid_timer = rospy.Timer(rospy.Duration(0.01), self.centroid_timer_cb, oneshot=False)
        self.track_object(timeout=-1, z_offset=0, object_name="centroid_des")

    def moveInFormation(self, goal, avg_spd):
        c = self.calculateCentroid().state.pose.position
        curr_centroid = np.array([[c.x, c.y, c.z]]).T
        ic = np.zeros((3,3))
        fc = np.zeros((3,3))

        goal = np.array([goal]).T
        waypoints = np.concatenate((curr_centroid, goal), 1)
        avg_spd = avg_spd

        self.traj = MinimumSnap(waypoints, ic, fc, avg_spd)
        self.t0 = rospy.get_time()

        self.track_object(timeout=-1, z_offset=0, object_name="centroid_des")

    def trackTargetInFormation(self, time: float, z_offset: float):
        horizon = 0.1

        t0 = rospy.get_time()

        sleeper = rospy.Rate(1/horizon)      
        self.track_object(timeout=-1, z_offset=-10, object_name="centroid_des")
  
        while not rospy.is_shutdown() and rospy.get_time() - t0 < time:

            tp = self.target.state.state.pose.position
            tv = self.target.state.state.vel.linear
            ta = self.target.state.state.acc.linear

            target_pos = np.array([[tp.x, tp.y, tp.z]]).T
            target_vel = np.array([[tv.x, tv.y, tv.z]]).T
            target_acc = np.array([[ta.x, ta.y, ta.z]]).T

            bias = target_vel*horizon + 0.5*target_acc*horizon**2
            fv = target_vel + target_acc*horizon 
            fa = target_acc
            fj = np.zeros((3,1))

            c = self.calculateCentroid().state.pose.position
            cen_pos = np.array([[c.x, c.y, c.z]]).T
            goal = target_pos + bias + np.array([[0,0,z_offset]]).T

            waypoints = np.concatenate((cen_pos, goal), 1)

            target_spd = np.linalg.norm(target_vel)
            d = np.linalg.norm((cen_pos - target_pos))

            if d < 0.125:
                spd_gain = 0
            else:
                spd_gain = 0.25
            
            avg_spd = target_spd + spd_gain*d
            avg_spd = np.minimum(avg_spd, 3.0)
            avg_spd = np.maximum(avg_spd, 0.1)


            cen_vel = self.calculateCentroid().state.vel.linear
            cen_acc = self.calculateCentroid().state.acc.linear

            iv = np.array([[cen_vel.x, cen_vel.y, cen_vel.z]]).T
            ia = np.array([[cen_acc.x, cen_acc.y, cen_acc.z]]).T
            ij = np.zeros((3,1))
            ic = np.concatenate([iv,ia,ij], 1).T

            fc = np.concatenate([fv,fa,fj], 1).T                

            self.traj = MinimumSnap(waypoints, ic, fc, avg_spd)
            sleeper.sleep()
        

    def deactivateAgents(self):
        if self.centroid_timer != None:
            self.centroid_timer.shutdown()
        
        self.centroid_timer = None


    def centroid_timer_cb(self, event):
        # Handle desired centroid
        (desiredState, accel) = self.traj.compute(rospy.get_time()-self.t0, np.zeros((10,1)), compute_control=False)

        msg = Multirotor()

        msg.state.pose.position = Point(*desiredState[0:3])
        msg.state.vel.linear = Vector3(*desiredState[7:10])
        msg.state.acc.linear = Vector3(*accel)

        self.centroid_des_pub.publish(msg)

        # Handle actual centroid
        msg = self.calculateCentroid()
        if self.goal == None:
            self.centroid_act_pub.publish(msg)
            return

        elif type(self.goal) != Multirotor:
            print("Unrecognized goal type: " + type(self.goal))
            return

    def getDroneList(self) -> Dict[str, Agent]:
        """
        Gets the dictionary of Agents where key is drone name and value is the agent process

        Returns:
            Dict[str, Agent]: Dictionary of Agents
        """
        return self.drones

    def takeoff(self, wait: bool = False) -> None:
        """
        Send the takeoff command to all agents.

        Args:
            wait (bool, optional): Wait for all drones to complete task. Defaults to False.
        """
        for i in self.vehicle_list:
            try:
                self.drones[i].services["takeoff"](False)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        if wait:
            self.wait()

    def land(self, wait: bool = False) -> None:
        """
        Send land command to all agents.

        Args:
            wait (bool, optional): Wait for all drones to complete task. Defaults to False.
        """
        for i in self.vehicle_list:
            try:
                self.drones[i].services["land"](False)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        # Doesn't Work?
        if wait:
            self.wait()

    def wait(self) -> None:
        for drone in self.vehicle_list:
            for action in self.drones[drone].actions.values():
                if action.get_state() != GoalStatus.LOST:
                    action.wait_for_result()


    def move_to_location(
        self, target: List[float], speed: float, timeout: float, tolerance: float, position_frame = MoveToLocationGoal.GLOBAL_FRAME, yaw_frame = MoveToLocationGoal.GLOBAL_FRAME
    ) -> None:
        """
        Move agents to location in a circle configuration.

        Args:
            target (List[float]): the center location to move the agents to
            timeout (float): the timeout to stop moving
            tolerance (float): the tolerance for the final position
            position_frame (int16): frame of reference for posiion command
            yaw_frame (int16): frame of reference for yaw command
        """

        l = 4 * math.pi / 3

        delta_theta = 2 * math.pi / len(self.vehicle_list)

        if len(self.vehicle_list) > 1:
            r = l / delta_theta
        else:
            r = 0

        i = 0

        for drone in self.vehicle_list:
            position = []
            position.append(target[0] - r * math.cos(delta_theta * i))
            position.append(target[1] - r * math.sin(delta_theta * i))
            position.append(target[2])

            yaw = 0.0

            i += 1

            goal = MoveToLocationGoal(
                target=position,
                position_frame=position_frame,
                timeout=timeout,
                speed = speed,
                tolerance=tolerance,
                yaw_frame=yaw_frame,
                yaw=yaw
            )
            self.drones[drone].actions["move_to_location"].send_goal(goal)

        for drone in self.vehicle_list:
            self.drones[drone].actions["move_to_location"].wait_for_result()

    def track_object(self, timeout: float, z_offset: float, object_name = "") -> None:
        """
        Commands the agents to track a target

        Args:
            object_name (str): the drone name to track, must be Target class
            timeout (float): timeout for the command
            z_offset (float): offset for the z axis
        """

        if object_name == "":
            target_name = self.target.name
        else:
            target_name = object_name

        # Track object in a circle configuration

        l = 4 * math.pi / 3

        delta_theta = 2 * math.pi / len(self.vehicle_list)

        if len(self.vehicle_list) > 1:
            r = l / delta_theta
        else:
            r = 0

        i = 0

        for drone in self.vehicle_list:
            dx = -r * math.cos(delta_theta * i)
            dy = -r * math.sin(delta_theta * i)
            dz = z_offset

            i += 1

            offset = (dx, dy, dz)
            goal = TrackObjectGoal(
                object_name=target_name+"/multirotor", timeout=timeout, offset=offset
            )
            self.drones[drone].actions["track"].send_goal(goal)

    def shutdown(self) -> None:
        """
        Shuts down the swarm
        """
        if self.__shutdown == True:
            return

        self.__shutdown = True

        if not rospy.is_shutdown():
            for i in self.vehicle_list:
                try:
                    self.drones[i].services["shutdown"](True)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

            try:
                if self.target != None:
                    resp = self.target.services["shutdown"](True)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)


if __name__ == "__main__":

    team = Team("team", ["Drone0", "Drone1"], None)

    ######################################
    #
    #     SETUP ROS

    rospy.init_node("swarm")
    rospy.on_shutdown(team.shutdown)

    team.setup_ros()
    rospy.sleep(1)


    #team.move_to_location([-240,-250,-5], speed=2, timeout=10, tolerance=0.1)
    team.activateAgents()
    rospy.sleep(5)

    team.moveInFormation([-250,-250,-5], 1.5)

    rospy.sleep(20)


    team.shutdown()