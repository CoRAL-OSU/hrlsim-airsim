#! /usr/bin/python3

import time
import multiprocessing as mp
import sys

import airsim
from airsim.client import MultirotorClient
from airsim.types import MultirotorState, LandedState
import rospy


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


class Drone(mp.Process):
    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: mp.Lock,
    ) -> None:
        mp.Process.__init__(self)

        self.__swarm_name = swarmName
        self.__drone_name = droneName

        self.__shutdown = False
        self.__service_timeout = 5.0  # SECONDS

        self.__client_lock = client_lock
        self.__flag_lock = mp.Lock()

        self.freq = 20

        with self.__client_lock:
            self.__client = sim_client
            self.__client.confirmConnection()
            self.__client.enableApiControl(True, vehicle_name=self.__drone_name)
            self.__client.armDisarm(True, vehicle_name=self.__drone_name)

        self.__vehicle_state = self.get_state()

    def __setup_ros(self) -> None:
        rospy.init_node(self.__drone_name)
        self.__topic_prefix = "/" + self.__swarm_name + "/" + self.__drone_name
        loop_time_topic = self.__topic_prefix + "/looptime"

        takeoff_service_name = self.__topic_prefix + "/takeoff"
        land_service_name = self.__topic_prefix + "/land"
        shutdown_service_name = self.__topic_prefix + "/shutdown"

        rospy.on_shutdown(self.shutdown)

        self.__looptime_pub = rospy.Publisher(loop_time_topic, Float32, queue_size=10)
        rospy.Service(shutdown_service_name, SetBool, self.__handle_shutdown)
        rospy.Service(takeoff_service_name, Takeoff, self.__handle_takeoff)
        rospy.Service(land_service_name, Land, self.__handle_land)

    def __handle_shutdown(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for the /shutdown rosservice. Uses Python API to disarm drone

        @param req (StdBoolRequest) Currently unused\n
        @return (StdBoolResponse) True on success
        """

        print("TARGET SHUTDOWN REQUEST RECEIVED")

        with self.__flag_lock:
            self.__finished = False
            self.__shutdown = True

            with self.__client_lock:
                self.__client.armDisarm(False, vehicle_name=self.__drone_name)
                self.__client.enableApiControl(False, vehicle_name=self.__drone_name)

            self.__finished = True

            print("TARGET SHUTDOWN REQUEST HANDLED")
            return SetBoolResponse(True, "")

    def __handle_takeoff(self, req: TakeoffRequest) -> TakeoffResponse:
        """
        Callback for the rosservice /takeoff. Uses the Python API to takeoff the drone

        @param req (TakeoffRequest) Request on rosservice
        @return (TakeoffResponse) True on successfuly takeoff. Else false.
        """

        with self.__flag_lock:
            self.__finished = False
            self.__cmd = None

            if self.get_state().landed_state == LandedState.Flying:
                self.__finished = True
                return TakeoffResponse(True)

            time_start = time.time()
            with self.__client_lock:
                self.__client.takeoffAsync(vehicle_name=self.__drone_name)

            if req.waitOnLastTask == False:
                self.__finished = True
                return TakeoffResponse(False)

            while (
                self.get_state().landed_state != LandedState.Flying
                and time.time() - time_start < self.__service_timeout
            ):
                time.sleep(0.05)

            self.__finished = True

            if self.__vehicle_state.landed_state == LandedState.Flying:
                return TakeoffResponse(True)
            else:
                return TakeoffResponse(False)

    def __handle_land(self, req: LandRequest) -> LandResponse:
        """
        Callback for the rosservice /land. Uses the Python API to land the drone.

        @param req (LandRequest) Request on rosservice \n
        @return (LandResponse) True of succesfully landed. Else false
        """

        with self.__flag_lock:
            self.__finished = False
            self.__cmd = None

            if self.get_state().landed_state == LandedState.Landed:
                self.__finished = True
                return LandResponse(True)

            time_start = time.time()
            with self.__client_lock:
                self.__client.landAsync(vehicle_name=self.__drone_name)

            if req.waitOnLastTask == False:
                self.__finished = True
                return LandResponse(False)

            while (
                self.get_state().landed_state != LandedState.Landed
                and time.time() - time_start < self.__service_timeout
            ):
                time.sleep(0.05)

            self.__finished = True

            if self.__vehicle_state.landed_state == LandedState.Landed:
                return LandResponse(True)
            else:
                return LandResponse(False)

    def shutdown(self):
        """
        Handle improper rospy shutdown. Uses Python API to disarm drone
        """
        with self.__flag_lock:
            self.__shutdown = True
            self.__finished = True

        print(self.__drone_name + " QUITTING")

    def get_state(self) -> MultirotorState:
        """
        Return the state of the drone. Refer to MultirotorClient.getMultirotorState for more information

        @return (MultirotorState)
        """

        with self.__client_lock:
            # Try to get state thrice
            error_count = 0
            max_errors = 3
            error = Exception()

            for _ in range(0, max_errors):
                try:
                    self.__vehicle_state = self.__client.getMultirotorState(
                        vehicle_name=self.__drone_name
                    )
                    break

                except Exception as e:
                    error = e
                    error_count += 1

            if error_count == max_errors:
                print(
                    self.__drone_name
                    + " Error from getMultirotorState API call: {0}".format(
                        error.message
                    )
                )

            for _ in range(0, max_errors):
                try:
                    pose = self.__client.simGetObjectPose(object_name=self.__drone_name)
                    break

                except Exception as e:
                    error = e
                    error_count += 1

            if error_count == max_errors:
                print(
                    self.__drone_name
                    + " Error from getMultirotorState API call: {0}".format(
                        error.message
                    )
                )

            self.__vehicle_state.kinematics_estimated.position = pose.position
            self.__vehicle_state.kinematics_estimated.orientation = pose.orientation

        return self.__vehicle_state

    def run(self) -> None:
        self.__setup_ros()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])

    client = airsim.MultirotorClient(ip="192.168.1.96")
    lock = mp.Lock()

    drone = Drone("swarm", drone_name, client, lock)
    drone.start()
    drone.join()
