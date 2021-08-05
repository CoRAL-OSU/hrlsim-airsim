#! /usr/bin/python3

from multiprocessing import Lock, Process
from math import cos, pi, sin, pow
from typing import List, Tuple

import rospy

from airsim_ros_cntrl.msg import Multirotor, State

from std_msgs.msg import Header, Float32
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from geometry_msgs.msg import (
    Twist,
    Pose,
    Point,
    Quaternion,
    Accel,
    Vector3
)

import hrlsim

class Target(Process):
    """
    Class to handle trackable targets for a team.
    A target is a drone meant to emulate a trackable object used to simulate the team.
    Refer to Drone documentation for more info.

    Args:
        swarmName (str): The name of the team this drone is associated with.
        droneName (str): The name of the drone itself.
        sim_client (MultirotorClient): The client to use to execture commands.
        client_lock (mp.Lock): The lock for the sim_client.
        path (List[Tuple[float, float, float]], optional): The path for the target to follow, if blank will be stationary. In format of linear velocity, angular velocity and time Defaults to [].
    """

    def __init__(
        self,
        swarmName: str,
        objectName: str,
        freq: int = 30,
        path: List[Tuple[float, float, float]] = [],
        ip="",
    ) -> None:
        """
        Constructs a new Target Process

        Args:
            swarmName (str): The name of the team this drone is associated with.
            droneName (str): The name of the drone itself.
            sim_client (MultirotorClient): The client to use to execture commands.
            client_lock (mp.Lock): The lock for the sim_client.
            path (List[Tuple[float, float, float]], optional): The path for the target to follow, if blank will be stationary. In format of linear velocity, angular velocity and time Defaults to [].
        """
        Process.__init__(self)

        self.swarm_name = swarmName
        self.drone_name = objectName
        self.freq = freq
        self.time_step = 1 / self.freq
        self.object_name = objectName
        self._shutdown = False
        self.flag_lock = Lock()
        self.prev_loop_time = -999
        self.linear_acc_rate = 0.5
        self.ang_acc_rate = 0.3

        self.client = hrlsim.airsim.MultirotorClient(ip)
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
        with self.flag_lock:
            self._shutdown = True
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
        state: hrlsim.airsim.MultirotorState
    ) -> None:
        """
        Function to publish sensor/state information from the simulator
        Returns: None
        """
        state = state.kinematics_estimated
        msg = Multirotor()
        
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.looptime = Float32(rospy.get_time() - self.prev_loop_time)
        self.prev_loop_time = rospy.get_time()


        # Setup pose msg
        pos = Point(*state.position.to_numpy_array())
        q = Quaternion(*state.orientation.to_numpy_array())      
        pose = Pose(pos,q)

        # Setup twist msg
        lin_vel = Vector3(*state.linear_velocity.to_numpy_array())
        ang_vel = Vector3(*state.angular_velocity.to_numpy_array())
        twist = Twist(lin_vel, ang_vel)

        # Setup acc msg
        lin_acc = Vector3(*state.linear_acceleration.to_numpy_array())
        ang_acc = Vector3(*state.angular_acceleration.to_numpy_array())
        acc = Accel(lin_acc, ang_acc)

        # Make state msg
        msg.state = State(pose, twist, acc)

        # Publish msg
        self.multirotor_pub.publish(msg)



    def generate_linear_velocity(
        self, current_velocity: float, position: hrlsim.airsim.Vector3r, waypoint: hrlsim.airsim.Vector3r
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
        if current_velocity-target_velocity > 0.1:
            a = -acceleration
        elif current_velocity-target_velocity < -0.1:
            a = acceleration
        else:
            a = 0
        v = current_velocity + a * self.time_step

        if (a == 0 and current_velocity < target_velocity) or (
            a == 0 and current_velocity > target_velocity
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

        rospy.sleep(1)

        traj_init_time = rospy.get_time()
        way_init_time = traj_init_time
        time = traj_init_time

        rate = rospy.Rate(self.freq)

        speed = 0

        v_yaw = 0

        angular_acc = 0

        linear_acc = 0

        way_lin, way_ang, way_time = self.path[self.path_index]
        traj_time = sum(row[2] for row in self.path)


        _, _, yaw = hrlsim.airsim.to_eularian_angles(self.pos.orientation)

        state = MultirotorState()

        print("Trajectory time: " + str(traj_time))

        while not rospy.is_shutdown() and self._shutdown == False and time - traj_init_time < traj_time:
            with self.flag_lock:
                if self._shutdown == True:
                    break


            if time - way_init_time >= way_time:
                self.path_index += 1
                if self.path_index == len(self.path):
                    way_lin = 0
                    way_ang = 0
                else:
                    way_lin, way_ang, way_time = self.path[self.path_index]
                    way_init_time = time

            v_yaw, angular_acc = self.generate_velocity(
                v_yaw, way_ang, self.ang_acc_rate
            )

            yaw_iter = v_yaw * self.time_step
            yaw += yaw_iter
            q = hrlsim.airsim.to_quaternion(0, 0, yaw)
            q = q / q.get_length()
            self.pos.orientation = q

            speed, linear_acc = self.generate_velocity(
                speed, way_lin, self.linear_acc_rate
            )
            v = hrlsim.airsim.Vector3r(speed * cos(yaw), speed * sin(yaw), 0)
            self.pos.position = v * self.time_step + self.pos.position

            self.client.simSetObjectPose(object_name=self.object_name, pose=self.pos)

            angular_acc_v = hrlsim.airsim.Vector3r(0, 0, angular_acc)
            v_yaw_v = hrlsim.airsim.Vector3r(0, 0, v_yaw)
            linear_acc_v = hrlsim.airsim.Vector3r(
                linear_acc * cos(yaw), linear_acc * sin(yaw), 0
            )

            state.kinematics_estimated.position = self.pos.position
            state.kinematics_estimated.orientation = self.pos.orientation
            state.kinematics_estimated.linear_velocity = v
            state.kinematics_estimated.linear_acceleration = linear_acc_v
            state.kinematics_estimated.angular_velocity = v_yaw_v
            state.kinematics_estimated.angular_acceleration = angular_acc_v

            self.publish_multirotor_state(state)

            rate.sleep()

            self.time_step = rospy.get_time() - time
            time = rospy.get_time()


if __name__ == "__main__":

    ip = "10.0.0.3"

    run_camera = True

    client = hrlsim.airsim.MultirotorClient(ip=ip)
    client.confirmConnection()


    target_list = [
        (
            "African_Poacher_1_WalkwRifleLow_Anim2_2",
            [(0,0,19), (0,0,7), (1.5, 0, 3.5), (1.5, -pi/10, 3), (1.5, -0.01, 10), (1.5,pi/5,1), (1.5, 0, 15)],
            hrlsim.airsim.Vector3r(-250, -312, 0),
            hrlsim.airsim.to_quaternion(0, 0, 2.32),
        ),
        (
            "African_Poacher_1_WalkwRifleLow_Anim3_11",
            [(0,0,19), (1.5, 0, 8), (1.5, -pi/10, 3.5), (1.5, -0.01, 10), (1.5,-pi/10,1), (1.5,0,15)],
            hrlsim.airsim.Vector3r(-259, -318, 0),
            hrlsim.airsim.to_quaternion(0, 0, 2.32),
        ),
    ]

    target_procs = []
    for t in target_list:
        client.simSetObjectPose(t[0], hrlsim.airsim.Pose(t[2],t[3]))
        target_procs.append(Target("test", t[0], path=t[1], ip=ip))
        target_procs[-1].start()

    if run_camera:
        from hrlsim.drone import Camera

        rospy.init_node("target_test")
        rospy.sleep(1)
        
        camera = Camera("Camera")
        camera.start()
        camera.join()


    for t in target_procs:
        t.join()