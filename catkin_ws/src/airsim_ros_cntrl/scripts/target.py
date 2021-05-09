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
        path (List[], optional): The path for the target to follow, if blank will be stationary. Defaults to [].
        path_type (str, optional): Generates a path for the target to follow. Possible values are 'Circle', 'Triangle', 'Square' and 'Line'. Defaults to "".
    """

    def __init__(
        self,
        swarmName: str,
        objectName: str,
        speed: float,
        freq: int = 20,
        path: List[Tuple[float, float, float]] = [],
        path_type: str = "",
        ip="",
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
        self.yaw_speed = 0.1
        self.ang_acc_rate = 0.08
        self.ang_acc_dis = -pow(self.yaw_speed, 2) / (2 * -self.ang_acc_rate)

        self.client = MultirotorClient(ip)
        self.client.confirmConnection()
        self.pos = self.client.simGetObjectPose(self.object_name)

        print("NEW TARGET: " + self.object_name)

        self.path = (
            Target.modify_path(path)
            if path_type == ""
            else self.generate_path(path_type)
        )
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

    def generate_path(self, path_type: str, radius=15, height=0) -> List[Vector3r]:
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
        center = self.pos.position

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

    def generate_angular_velocity(
        self, current_velocity: float, yaw: float, way_yaw: float
    ):
        d = abs(way_yaw - yaw)
        ang_acc_dis = -pow(current_velocity, 2) / (2 * -self.ang_acc_rate)
        if d <= ang_acc_dis:
            a = -self.ang_acc_rate
            v = current_velocity - self.ang_acc_rate * self.time_step
        elif current_velocity < self.yaw_speed:
            v = current_velocity + self.ang_acc_rate * self.time_step
            a = self.ang_acc_rate
        if current_velocity >= self.yaw_speed:
            a = 0
            v = self.yaw_speed
        return (v, a)

    def run(self) -> None:
        """
        @Override
        Funciton to run when the process starts.

        Returns: None
        """
        self.__setup_ros()

        # time.sleep(8)

        rate = rospy.Rate(self.freq)

        speed = 0

        v_yaw = 0

        angular_acc = 0

        linear_acc = 0

        waypoint = self.path[self.path_index]

        _, _, yaw = airsim.to_eularian_angles(self.pos.orientation)
        d: Vector3r = waypoint - self.pos.position
        way_yaw = atan2(d.y_val, d.x_val)
        if abs(way_yaw - yaw) > abs(way_yaw - (yaw + 2 * pi)):
            yaw += 2 * pi
        elif abs(way_yaw - yaw) > abs(way_yaw - (yaw - 2 * pi)):
            yaw -= 2 * pi
        v = airsim.Vector3r(self.speed * cos(way_yaw), self.speed * sin(way_yaw), 0)

        while not rospy.is_shutdown() and self._shutdown == False:
            with self.flag_lock:
                if self.shutdown == True:
                    break
            if abs(way_yaw - yaw) > 0.01:
                print(way_yaw - yaw, way_yaw, yaw)
                v_yaw, angular_acc = self.generate_angular_velocity(v_yaw, yaw, way_yaw)
                yaw_iter = v_yaw * self.time_step
                if way_yaw > yaw:
                    yaw += yaw_iter
                else:
                    yaw -= yaw_iter
                q = airsim.to_quaternion(0, 0, yaw)
                q = q / q.get_length()
                self.pos.orientation = q
            elif waypoint.distance_to(self.pos.position) < 1:
                self.path_index += 1
                if self.path_index == len(self.path):
                    break
                waypoint = self.path[self.path_index]
                d: Vector3r = waypoint - self.pos.position
                way_yaw = atan2(d.y_val, d.x_val)
                if abs(way_yaw - yaw) > abs(way_yaw - (yaw + 2 * pi)):
                    yaw += 2 * pi
                elif abs(way_yaw - yaw) > abs(way_yaw - (yaw - 2 * pi)):
                    yaw -= 2 * pi
            else:
                print(waypoint.distance_to(self.pos.position))
                speed, linear_acc = self.generate_linear_velocity(
                    speed, self.pos.position, waypoint
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

            rate.sleep()

        print(self.object_name + " QUITTING")


if __name__ == "__main__":

    client = airsim.MultirotorClient(ip="192.168.1.2")
    name = "African_Poacher_1_WalkwRifleLow_Anim2_2"
    pos = client.simGetObjectPose("African_Poacher_1_WalkwRifleLow_Anim2_2")
    pos.position = airsim.Vector3r(-235, -242, 0)
    client.simSetObjectPose(name, pos)

    drone = Target("test", name, 2, path_type="circle", ip="192.168.1.2")
    drone.start()

    drone.join()
