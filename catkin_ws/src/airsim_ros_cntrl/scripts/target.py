#! /usr/bin/python3

import multiprocessing as mp
import time
from math import cos, pi, sin
from typing import List, Tuple

from matplotlib.pyplot import flag
from airsim.client import MultirotorClient
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
from std_msgs.msg import Header
from std_msgs.msg import Float32
from drone import Drone


class Target(Drone):
    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: mp.Lock,
        path: List[Tuple[float, float, int]] = [],
        path_type: str = "",
    ) -> None:
        """
        Initialize a Drone process, spinup the ROS node, and setup topics/services/action servers

        All topic names are set up as /swarm_name/drone_name/major_cmd/minor_cmd. Eg. commanding the velocity
        of drone1 in swarm1 would be: /swarm1/drones1/cmd/vel

        @param swarmName (string) Name of the UAV swarm
        ---
        @param droneName (string) Name of the UAV
        ---
        @param sim_client (MultirotorClient) Airsim Python Client
        ---
        """
        super().__init__(swarmName, droneName, sim_client, client_lock)

        with self.__client_lock:
            self.__path_future = self.__client.takeoffAsync(
                vehicle_name=self.__drone_name
            )

        self.__path = path if path_type == "" else self.generate_path(path_type)
        self.__path_index = 0

    def __setup_ros(self) -> None:
        super().__setup_ros()

        vel_topic = self.__topic_prefix + "/vel"
        pos_topic = self.__topic_prefix + "/pos"
        acc_topic = self.__topic_prefix + "/acc"

        self.__pos_pub = rospy.Publisher(pos_topic, PoseStamped, queue_size=10)
        self.__vel_pub = rospy.Publisher(vel_topic, TwistStamped, queue_size=10)
        self.__acc_pub = rospy.Publisher(acc_topic, AccelStamped, queue_size=10)

    def generate_path(
        self, path_type, radius=5, height=-5
    ) -> List[Tuple[float, float, int]]:
        paths = {"circle": 12, "triangle": 3, "square": 4, "line": 2}
        points = paths[path_type]
        center = self.get_state().kinematics_estimated.position
        path: List[Tuple[float, float, int]] = []
        for i in range(points):
            x = radius * cos(pi / points * i * 2) + center.x_val
            y = radius * sin(pi / points * i * 2) + center.y_val
            path.append((x, y, height))
        return path

    def run(self) -> None:
        self.__setup_ros()

        time.sleep(8)

        rate = rospy.Rate(self.freq)

        prev_time = time.time()

        with self.__client_lock:
            self.__path_future = self.__client.moveToPositionAsync(
                *self.__path[self.__path_index], 2, 20, vehicle_name=self.__drone_name
            )
        self.__path_index += 1
        if self.__path_index == len(self.__path):
            self.__path_index = 0

        while not rospy.is_shutdown() and not self.__shutdown:
            with self.__flag_lock:
                if self.__shutdown == True:
                    break

            state = self.get_state().kinematics_estimated

            if self.__pos_pub:
                point = Point(
                    state.position.x_val, pose.position.y_val, pose.position.z_val
                )
                quat = Quaternion(
                    state.orientation.x_val,
                    pose.orientation.y_val,
                    pose.orientation.z_val,
                    pose.orientation.w_val,
                )
                pose = Pose(point, quat)
                header = Header()
                header.stamp = rospy.Time.now()

                msg = PoseStamped(header, pose)
                self.__pos_pub.publish(msg)

            if self.__vel_pub:
                linear = Vector3(
                    state.linear_velocity.x_val,
                    state.linear_velocity.y_val,
                    state.linear_velocity.z_val,
                )
                angular = Vector3(
                    state.angular_velocity.x_val,
                    state.angular_velocity.y_val,
                    state.angular_velocity.z_val,
                )
                twist = Twist(linear, angular)
                header = Header()
                header.stamp = rospy.Time.now()
                msg = TwistStamped(header, twist)
                self.__vel_pub.publish(msg)

            if self.__acc_pub:
                linear = Vector3(
                    state.linear_acceleration.x_val,
                    state.linear_acceleration.y_val,
                    state.linear_acceleration.z_val,
                )
                angular = Vector3(
                    state.angular_acceleration.x_val,
                    state.angular_acceleration.y_val,
                    state.angular_acceleration.z_val,
                )
                accel = Accel(linear, angular)
                header = Header()
                header.stamp = rospy.Time.now()
                msg = AccelStamped(header, accel)
                self.__acc_pub.publish(msg)

            if self.__looptime_pub:
                elapsed_time = time.time() - prev_time
                prev_time = time.time()

                msg = Float32(elapsed_time)
                self.__looptime_pub.publish(msg)

            rate.sleep()

        print(self.__drone_name + " QUITTING")
