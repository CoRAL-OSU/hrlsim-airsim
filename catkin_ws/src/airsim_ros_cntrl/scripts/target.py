#! /usr/bin/python3

from multiprocessing import Lock, Process
import time
from math import cos, pi, sin, atan2, pow
from typing import List, Tuple
from airsim.types import Quaternionr
import sys

from airsim.client import MultirotorClient
from airsim import Vector3r
import airsim
import rospy

from airsim_ros_cntrl.msg import Multirotor, State
from geometry_msgs.msg import Twist, Pose, Accel
from std_msgs.msg import Header, Float32
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest


class Target(Process):
    """
    Class to handle trackable targets for a team.
    A target is a drone meant to emulate a trackable object used to simulate the team.
    Refer to Drone documentation for more info.

    Args:
        swarmName (str): The name of the team this drone is associated with.
        droneName (str): The name of the drone itself.
        sim_client (airsim.MultirotorClient): The client to use to execture commands.
        client_lock (mp.Lock): The lock for the sim_client.
        path (List[Tuple[float, float, float]], optional): The path for the target to follow, if blank will be stationary. In format of linear velocity, angular velocity and time Defaults to [].
    """

    def __init__(
        self,
        swarmName: str,
        objectName: str,
        speed: float,
        freq: int = 20,
        path: List[Tuple[float, float, float]] = [],
        ip="",
    ) -> None:
        """
        Constructs a new Target Process

        Args:
            swarmName (str): The name of the team this drone is associated with.
            droneName (str): The name of the drone itself.
            sim_client (airsim.MultirotorClient): The client to use to execture commands.
            client_lock (mp.Lock): The lock for the sim_client.
            path (List[Tuple[float, float, float]], optional): The path for the target to follow, if blank will be stationary. In format of linear velocity, angular velocity and time Defaults to [].
        """
        Process.__init__(self)

        self.speed = speed
        self.swarm_name = swarmName
        self.freq = freq
        self.time_step = 1 / self.freq
        self.object_name = objectName
        self._shutdown = False
        self.flag_lock = Lock()
        self.prev_loop_time = time.time()
        self.linear_acc_rate = 0.1
        self.ang_acc_rate = 0.08

        self.client = MultirotorClient(ip)
        self.client.confirmConnection()
        self.pos = self.client.simGetObjectPose(self.object_name)

        print("NEW TARGET: " + self.object_name)

        self.path = path
        self.path_index = 0

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

        Returns: None
        """
        rospy.init_node(self.object_name)
        self.topic_prefix = "/" + self.swarm_name + "/" + self.object_name
        shutdown_service_name = self.topic_prefix + "/shutdown"
        state_topic = self.topic_prefix + "/multirotor"
        rospy.on_shutdown(self.shutdown)

        rospy.Service(shutdown_service_name, SetBool, self.__handle_shutdown)
        self.multirotor_pub = rospy.Publisher(state_topic, Multirotor, queue_size=10)

    def __handle_shutdown(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for the /shutdown rosservice. Uses Python API to disarm drone

        Args:
            req (StdBoolRequest): Currently unused

        Returns (StdBoolResponse): True on success
        """

        print(self.object_name + " SHUTDOWN REQUEST RECEIVED")
        with self.flag_lock:
            self._shutdown = True

            print(self.object_name + " SHUTDOWN REQUEST HANDLED")
            return SetBoolResponse(True, "")

    def shutdown(self) -> None:
        """
        Handle improper rospy shutdown. Uses Python API to disarm drone

        Returns: None
        """
        with self.flag_lock:
            self._shutdown = True

        print(self.object_name + " QUITTING")

    def publish_multirotor_state(
        self,
        pos: Pose,
        vel: Vector3r,
        ang_vel: Vector3r,
        linear_acc: Vector3r,
        ang_acc: Vector3r,
    ) -> None:
        """
        Function to publish sensor/state information from the simulator
        Returns: None
        """

        msg = Multirotor()

        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.looptime = Float32(time.time() - self.prev_loop_time)
        self.prev_loop_time = time.time()

        twist = Twist(vel, ang_vel)

        acc = Accel(linear_acc, ang_acc)

        # Make state msg
        msg.state = State(pos, twist, acc)

        # Publish msg
        self.multirotor_pub.publish(msg)

    def generate_linear_velocity(
        self, current_velocity: float, position: Vector3r, waypoint: Vector3r
    ):
        d = waypoint.distance_to(position)
        linear_acc_dis = -pow(current_velocity, 2) / (2 * -self.linear_acc_rate)
        if d <= linear_acc_dis:
            a = -self.linear_acc_rate
            v = current_velocity - self.linear_acc_rate * self.time_step
        elif current_velocity < self.speed:
            v = current_velocity + self.linear_acc_rate * self.time_step
            a = self.linear_acc_rate
        if current_velocity >= self.speed:
            a = 0
            v = self.speed
        return (v, a)

    def generate_velocity(
        self, current_velocity: float, target_velocity: float, acceleration: float
    ):
        if current_velocity > target_velocity:
            a = -acceleration
        elif current_velocity < target_velocity:
            a = acceleration
        else:
            a = 0
        v = current_velocity + a * self.time_step
        if (a < 0 and current_velocity < target_velocity) or (
            a > 0 and current_velocity > target_velocity
        ):
            v = target_velocity

        return (v, a)

    def run(self) -> None:
        """
        @Override
        Funciton to run when the process starts.

        Returns: None
        """
        self.__setup_ros()

        time = 0

        rate = rospy.Rate(self.freq)

        speed = 0

        v_yaw = 0

        angular_acc = 0

        linear_acc = 0

        way_lin, way_ang, way_time = self.path[self.path_index]

        _, _, yaw = airsim.to_eularian_angles(self.pos.orientation)

        while not rospy.is_shutdown() and self._shutdown == False:
            with self.flag_lock:
                if self._shutdown == True:
                    break
            if time is not None and time >= way_time:
                self.path_index += 1
                if self.path_index == len(self.path):
                    way_lin = 0
                    way_ang = 0
                    time = None
                else:
                    way_lin, way_ang, way_time = self.path[self.path_index]
                    time = 0

            v_yaw, angular_acc = self.generate_velocity(
                v_yaw, way_ang, self.ang_acc_rate
            )

            yaw_iter = v_yaw * self.time_step
            yaw += yaw_iter
            q = airsim.to_quaternion(0, 0, yaw)
            q = q / q.get_length()
            self.pos.orientation = q

            speed, linear_acc = self.generate_velocity(
                speed, way_lin, self.linear_acc_rate
            )
            v = airsim.Vector3r(speed * cos(yaw), speed * sin(yaw), 0)
            self.pos.position = v * self.time_step + self.pos.position

            self.client.simSetObjectPose(object_name=self.object_name, pose=self.pos)

            angular_acc_v = Vector3r(0, 0, angular_acc)
            v_yaw_v = Vector3r(0, 0, v_yaw)
            linear_acc_v = airsim.Vector3r(
                linear_acc * cos(yaw), linear_acc * sin(yaw), 0
            )
            self.publish_multirotor_state(
                self.pos, v, v_yaw_v, linear_acc_v, angular_acc_v
            )

            if speed == 0 and v_yaw == 0:
                break

            if time is not None:
                time += self.time_step
            rate.sleep()

        print(self.object_name + " QUITTING")


if __name__ == "__main__":

    client = airsim.MultirotorClient(ip="192.168.1.2")
    name = "African_Poacher_1_WalkwRifleLow_Anim2_2"
    pos = client.simGetObjectPose("African_Poacher_1_WalkwRifleLow_Anim2_2")
    pos.position = airsim.Vector3r(-235, -242, 0)
    pos.orientation = airsim.Vector3r(0, 0, 0)
    client.simSetObjectPose(name, pos)

    drone = Target(
        "test", name, 2, path=[(2, -0.5, 15), (1, 0.5, 15)], ip="192.168.1.2"
    )
    drone.start()

    drone.join()
