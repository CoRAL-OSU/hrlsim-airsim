#! /usr/bin/python3

import time
from multiprocessing import Lock, Process
import sys

import airsim
from airsim.client import MultirotorClient
from airsim.types import MultirotorState, LandedState
import rospy
from actionlib import SimpleActionClient
from typing import Dict
from airsim_ros_pkgs.srv import (
    Takeoff,
    TakeoffRequest,
    TakeoffResponse,
    Land,
    LandRequest,
    LandResponse,
)

from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from std_msgs.msg import Float32


class DroneInfo:
    """
    Class to manage drone info.

    Args:
        DroneName (str): The name of the drone
        process (Drone): Process for a Drone
        pubs (Dict[str, rospy.Publisher], optional): Dict of publishers associated with a drone. Defaults to None.
        subs (Dict[str, rospy.Subscriber], optional): Dict of subscribers associated with a drone. Defaults to None.
        services (Dict[str, rospy.ServiceProxy], optional): Dict of services associated with a drone. Defaults to None.
        actions (Dict[str, SimpleActionClient], optional): Dict of actions associated with a drone. Defaults to None.
    """
    def __init__(self, DroneName: str, process: Drone, pubs: Dict[str, rospy.Publisher]=None, subs: Dict[str, rospy.Subscriber]=None, services: Dict[str, rospy.ServiceProxy]=None, actions: Dict[str, SimpleActionClient]=None):
        """
        Constructs info about a drone.
        Used in teams to keep track of all the different ros topics

        Args:
            DroneName (str): The name of the drone
            process (Drone): Process for a Drone
            pubs (Dict[str, rospy.Publisher], optional): Dict of publishers associated with a drone. Defaults to None.
            subs (Dict[str, rospy.Subscriber], optional): Dict of subscribers associated with a drone. Defaults to None.
            services (Dict[str, rospy.ServiceProxy], optional): Dict of services associated with a drone. Defaults to None.
            actions (Dict[str, SimpleActionClient], optional): Dict of actions associated with a drone. Defaults to None.
        """
        self.name = DroneName
        self.process = process
        self.pubs = pubs
        self.subs = subs
        self.services = services
        self.actions = actions

class Drone(Process):
    """
    Super Class for a generalized drone process.
    Each drone has a ros node and generalize topics and services for all types of drones

    All topic names are set up as /swarm_name/drone_name/major_cmd/minor_cmd. Eg. commanding the velocity
    of drone1 in swarm1 would be: /swarm1/drones1/cmd/vel

    All commands are executed through a singluar sim_client that all drones share, therefore, you must aquire the client_lock before using the client.

    Args:
        swarmName (str): The name of the swarm this drone is associated with.
        droneName (str): The name of the drone itself.
        sim_client (airsim.MultirotorClient): The client to use to execture commands.
        client_lock (mp.Lock): The lock for the sim_client.
    """
    
    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: Lock,
    ) -> None:
        """
        Constructs a new Drone Process.

        Args:
            swarmName (str): The name of the swarm this drone is associated with.
            droneName (str): The name of the drone itself.
            sim_client (airsim.MultirotorClient): The client to use to execture commands.
            client_lock (mp.Lock): The lock for the sim_client.
        """
        super().__init__(self)

        self.swarm_name = swarmName
        self.drone_name = droneName

        self._shutdown = False
        self.__service_timeout = 5.0  # SECONDS

        self.client_lock = client_lock
        self.flag_lock = Lock()

        self.freq = 20

        self.cmd = None

        with self.client_lock:
            self.client = sim_client
            self.client.confirmConnection()
            self.client.enableApiControl(True, vehicle_name=self.drone_name)
            self.client.armDisarm(True, vehicle_name=self.drone_name)

        self.__vehicle_state = self.get_state()

    def setup_ros(self) -> None:
        """
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Topics:
            /looptime (Float32): The looptime of the drone

        Services:
            /takeoff (airsim_ros_pkgs.Takeoff): Service to takeoff the drone
            /land (airsim_ros_pkgs.Land): Service to land the drone
            /shutdown (std_srvs.SetBool): Service to shutdown process

        Returns: None
        """
        rospy.init_node(self.drone_name)
        self.topic_prefix = "/" + self.swarm_name + "/" + self.drone_name
        loop_time_topic = self.topic_prefix + "/looptime"

        takeoff_service_name = self.topic_prefix + "/takeoff"
        land_service_name = self.topic_prefix + "/land"
        shutdown_service_name = self.topic_prefix + "/shutdown"

        rospy.on_shutdown(self.shutdown)

        self.looptime_pub = rospy.Publisher(loop_time_topic, Float32, queue_size=10)
        rospy.Service(shutdown_service_name, SetBool, self.__handle_shutdown)
        rospy.Service(takeoff_service_name, Takeoff, self.__handle_takeoff)
        rospy.Service(land_service_name, Land, self.__handle_land)

    def __handle_shutdown(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for the /shutdown rosservice. Uses Python API to disarm drone

        Args:
            req (StdBoolRequest): Currently unused

        Returns (StdBoolResponse): True on success
        """

        print("TARGET SHUTDOWN REQUEST RECEIVED")

        with self.flag_lock:
            self._shutdown = True

            with self.client_lock:
                self.client.armDisarm(False, vehicle_name=self.drone_name)
                self.client.enableApiControl(False, vehicle_name=self.drone_name)


            print("TARGET SHUTDOWN REQUEST HANDLED")
            return SetBoolResponse(True, "")

    def __handle_takeoff(self, req: TakeoffRequest) -> TakeoffResponse:
        """
        Callback for the rosservice /takeoff. Uses the Python API to takeoff the drone

        Args:
            req (TakeoffRequest): Request on rosservice

        Returns (TakeoffResponse): True on successfuly takeoff. Else false.
        """
        with self.flag_lock:
            self.cmd = None

            if self.get_state().landed_state == LandedState.Flying:
                return TakeoffResponse(True)

            time_start = time.time()
            with self.client_lock:
                self.client.takeoffAsync(vehicle_name=self.drone_name)

            if req.waitOnLastTask == False:
                return TakeoffResponse(False)

            while (
                self.get_state().landed_state != LandedState.Flying
                and time.time() - time_start < self.__service_timeout
            ):
                time.sleep(0.05)

            if self.__vehicle_state.landed_state == LandedState.Flying:
                return TakeoffResponse(True)
            else:
                return TakeoffResponse(False)

    def __handle_land(self, req: LandRequest) -> LandResponse:
        """
        Callback for the rosservice /land. Uses the Python API to land the drone.

        Args:
            req (LandRequest): Request on rosservice
        
        Returns (LandResponse): True of succesfully landed. Else false
        """

        with self.flag_lock:
            self.cmd = None

            if self.get_state().landed_state == LandedState.Landed:
                return LandResponse(True)

            time_start = time.time()
            with self.client_lock:
                self.client.landAsync(vehicle_name=self.drone_name)

            if req.waitOnLastTask == False:
                return LandResponse(False)

            while (
                self.get_state().landed_state != LandedState.Landed
                and time.time() - time_start < self.__service_timeout
            ):
                time.sleep(0.05)


            if self.__vehicle_state.landed_state == LandedState.Landed:
                return LandResponse(True)
            else:
                return LandResponse(False)

    def shutdown(self) -> None:
        """
        Handle improper rospy shutdown. Uses Python API to disarm drone

        Returns: None
        """
        with self.flag_lock:
            self._shutdown = True

        print(self.drone_name + " QUITTING")

    def get_state(self) -> MultirotorState:
        """
        Return the state of the drone. Refer to MultirotorClient.getMultirotorState for more information.
        Modifies the kinematics_estimated position and orientation to be updated to global frame.

        Returns (MultirotorState): The state of the drone.
        """

        with self.client_lock:
            # Try to get state thrice
            error_count = 0
            max_errors = 3
            error = Exception()

            for _ in range(0, max_errors):
                try:
                    self.__vehicle_state = self.client.getMultirotorState(
                        vehicle_name=self.drone_name
                    )
                    break

                except Exception as e:
                    error = e
                    error_count += 1

            if error_count == max_errors:
                print(
                    self.drone_name
                    + " Error from getMultirotorState API call: {0}".format(
                        error.message
                    )
                )

            for _ in range(0, max_errors):
                try:
                    pose = self.client.simGetObjectPose(object_name=self.drone_name)
                    break

                except Exception as e:
                    error = e
                    error_count += 1

            if error_count == max_errors:
                print(
                    self.drone_name
                    + " Error from getMultirotorState API call: {0}".format(
                        error.message
                    )
                )
            else:
                self.__vehicle_state.kinematics_estimated.position = pose.position
                self.__vehicle_state.kinematics_estimated.orientation = pose.orientation

        return self.__vehicle_state

    def run(self) -> None:
        """
        Funciton to run when the process starts.

        Returns: None
        """
        self.setup_ros()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])

    client = airsim.MultirotorClient(ip="192.168.1.96")
    lock = Lock()

    drone = Drone("swarm", drone_name, client, lock)
    drone.start()
    drone.join()
