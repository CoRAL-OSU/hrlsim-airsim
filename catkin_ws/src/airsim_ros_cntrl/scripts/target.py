#! /usr/bin/python3

from multiprocessing import Lock
import time
from math import cos, pi, sin
from typing import List, Tuple

from airsim.client import MultirotorClient
from airsim import Vector3r
import rospy

from geometry_msgs.msg import (
    TwistStamped,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist,
    Vector3,
    AccelStamped,
    Accel,
)
from std_msgs.msg import Header, Float32
from drone import Drone


class Target(Drone):
    """
    Class to handle trackable targets for a team.
    A target is a drone meant to emulate a trackable object used to simulate the team.
    Refer to Drone documentation for more info.

    Args:
        swarmName (str): The name of the team this drone is associated with.
        droneName (str): The name of the drone itself.
        sim_client (airsim.MultirotorClient): The client to use to execture commands.
        client_lock (mp.Lock): The lock for the sim_client.
        path (List[], optional): The path for the target to follow, if blank will be stationary. Defaults to [].
        path_type (str, optional): Generates a path for the target to follow. Possible values are 'Circle', 'Triangle', 'Square' and 'Line'. Defaults to "".
    """

    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: Lock,
        speed: float,
        path: List[Tuple[float, float, float]] = [],
        path_type: str = "",
    ) -> None:
        """
        Constructs a new Target Process

        Args:
            swarmName (str): The name of the team this drone is associated with.
            droneName (str): The name of the drone itself.
            sim_client (airsim.MultirotorClient): The client to use to execture commands.
            client_lock (mp.Lock): The lock for the sim_client.
            path (List[Tuple[float, float, float]], optional): The path for the target to follow, if blank will be stationary. Defaults to [].
            path_type (str, optional): Generates a path for the target to follow. Possible values are 'Circle', 'Triangle', 'Square' and 'Line'. Defaults to "".
        """
        super().__init__(swarmName, droneName, sim_client, client_lock)

        self.speed = speed


        print("NEW TARGET: " + self.drone_name)

        with self.client_lock:
            self.client.takeoffAsync(vehicle_name=self.drone_name)

        self.__path = Target.modify_path(path) if path_type == "" else self.generate_path(path_type)

    def __setup_ros(self) -> None:
        """
        @Override
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Topics:
            /looptime (Float32): The looptime of the drone
            /vel (geometry_msgs.TwistStamped): Velocity of the target.
            /pos (geometry_msgs.PoseStamped): Position of the target in global frame NED.
            /acc (geometry_msgs.AccelStamped): Acceleration of the target.

        Services:
            /takeoff (airsim_ros_pkgs.Takeoff): Service to takeoff the drone
            /land (airsim_ros_pkgs.Land): Service to land the drone
            /shutdown (std_srvs.SetBool): Service to shutdown process

        Returns: None
        """
        super().setup_ros()


    def generate_path(self, path_type: str, radius=10, height=-5) -> List[Vector3r]:
        """
        Generates a default path for the target to follow.

        Args:
            path_type (str): Shape for a target to follow. Possible values are 'Circle', 'Triangle', 'Square' and 'Line'.
            radius (int, optional): Radius of the shape. Defaults to 5.
            height (int, optional): Height of the path. Defaults to -5.

        Returns:
            List[Vector3r]: List of Points in NED orientation for the drone to follow.
        """
        paths = {"circle": 12, "triangle": 3, "square": 4, "line": 2, "f": 0}
        points = paths[path_type]
        center = self.state.kinematics_estimated.position

        path: List[Vector3r] = []
        for i in range(points):
            x = radius * cos(pi / points * i * 2) + center.x_val
            y = radius * sin(pi / points * i * 2) + center.y_val
            path.append(Vector3r(x, y, height))

        return path

    @staticmethod
    def modify_path(path: List[Tuple[float, float, int]]) -> List[Vector3r]:
        """
        Static method to translate the path from tuple points into Vectors. Used to feed into the MoveOnPath.

        Args:
            path (List[Tuple[float, float, int]]): Path to translate

        Returns:
            List[Vector3r]: Vector Path
        """
        new_path: List[Vector3r] = []
        for p in path:
            new_path.append(Vector3r(*p))
        return new_path


    def run(self) -> None:
        """
        @Override
        Funciton to run when the process starts.

        Returns: None
        """
        self.__setup_ros()

        time.sleep(8)

        rate = rospy.Rate(self.freq)

        prev_time = time.time()

        with self.client_lock:
            self.client.moveOnPathAsync(self.__path, self.speed, vehicle_name=self.drone_name)

        while not rospy.is_shutdown() and self._shutdown == False:
            with self.flag_lock:
                if self._shutdown == True:
                    break

            state = self.state.kinematics_estimated

            if (state.linear_velocity.get_length() < 0.2):
                self.client.moveOnPathAsync(self.__path, self.speed, vehicle_name=self.drone_name)

        
            self.publish_multirotor_state(self.state, self.sensors)

            rate.sleep()

        print(self.drone_name + " QUITTING")
