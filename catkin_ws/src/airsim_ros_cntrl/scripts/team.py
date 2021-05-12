#! /usr/bin/python3

from multiprocessing import Lock

import math

from airsim.client import MultirotorClient

import rospy, actionlib

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import TwistStamped, PoseStamped
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
        target: Target,
        client: MultirotorClient,
        lock: Lock,
    ) -> None:
        """
        Contructs a team object.
        A team is comprised of any number of agents and up to one target to track. 

        Args:
            teamName (str): The name of the team
            vehicle_list (List[str]): List of agent names
            target (Target): The target to track
            client (MultirotorClient): The client to use
            lock (Lock): The client lock
        """
        self.client = client
        self.lock = lock

        self.team_name = teamName

        self.vehicle_list = vehicle_list

        self.drones: Dict[str, Agent] = dict()

        if target != None:
            self.target = DroneInfo(target.drone_name, target)
        else:
            self.target = None

        self.__shutdown = False

        for i in self.vehicle_list:
            proc = Agent(self.team_name, i, self.client, self.lock)

            self.drones[i] = DroneInfo(i, proc)
            self.drones[i].process.start()

        print("SWARM CREATED WITH %d DRONES" % len(self.drones))

    def setup_ros(self) -> None:
        """
        Function to set up ros topics to talk to teams.
        The team itself does not have a node associated with it.
        """
        for i in self.vehicle_list:

            prefix = "/" + self.team_name + "/" + i

            cmd_vel_topic_name = prefix + "/cmd/vel"
            cmd_pos_topic_name = prefix + "/cmd/pos"

            pubs = dict()
            pubs["cmd_vel"] = rospy.Publisher(
                cmd_vel_topic_name, TwistStamped, queue_size=10
            )
            pubs["cmd_pos"] = rospy.Publisher(
                cmd_pos_topic_name, PoseStamped, queue_size=10
            )

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
            # move_to_location_action_name = prefix + "/move_to_location"

            actions = dict()
            actions["track"] = actionlib.SimpleActionClient(
                track_object_action_name, TrackObjectAction
            )
            actions["track"].wait_for_server()

            # actions["move_to_location"] = actionlib.SimpleActionClient(
            #    move_to_location_action_name, MoveToLocationAction
            # )
            # actions["move_to_location"].wait_for_server()

            self.drones[i].pubs = pubs
            self.drones[i].services = srvs
            self.drones[i].actions = actions

            if self.target != None:
                target_prefix = "/" + self.team_name + "/" + self.target.name
                # subs = dict()
                # subs['pos'] = rospy.Subscriber(target_prefix+"/pos", PoseStamped, )

                srvs = dict()
                srvs["shutdown"] = rospy.ServiceProxy(
                    target_prefix + "/shutdown", SetBool
                )
                self.target.services = srvs

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

    def cmd_pos(
        self,
        cmd: PoseStamped = None,
        cmd_all: PoseStamped = None,
        wait: bool = False,
        drone_name: str = None,
    ) -> None:
        """
        Send command to agents to move to position. Can either send same position to all via cmd_all or to one drone by specifying cmd and drone_name.

        Args:
            cmd (PoseStamped, optional): Command to send to a single agent. Must also specify drone_name. Defaults to None.
            cmd_all (PoseStamped, optional): Command to send to all agents. Defaults to None.
            wait (bool, optional): Toggle to wait for all drones to complete. Defaults to False.
            drone_name (str, optional): Drone name to send a single command to. Defaults to None.
        """
        if cmd_all != None:
            for i in self.vehicle_list:
                self.drones[i].pubs["cmd_pos"].publish(cmd_all)

        else:
            self.drones[drone_name].pubs["cmd_pos"].publish(cmd)

        # Doesn't Work?
        if wait:
            self.wait()

    def move_to_location(
        self, target: List[float], timeout: float, tolerance: float
    ) -> None:
        """
        Move agents to location in a circle configuration.

        Args:
            target (List[float]): the center location to move the agents to
            timeout (float): the timeout to stop moving
            tolerance (float): the tolerance for the final position
        """

        l = 8 * math.pi / 3

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

            yaw_frame = "local"
            yaw = 0.0

            i += 1

            goal = MoveToLocationGoal(
                target=position,
                timeout=timeout,
                tolerance=tolerance,
                yaw_frame=yaw_frame,
                yaw=yaw,
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
                object_name=target_name, timeout=timeout, offset=offset
            )
            self.drones[drone].actions["track"].send_goal(goal)

    def shutdown(self) -> None:
        """
        Shuts down the swarm
        """
        print("SHUTDOWN SWARM")

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

        with self.lock:
            self.client.cancelLastTask()