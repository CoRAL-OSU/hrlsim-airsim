#! /usr/bin/python3

import time
import multiprocessing as mp
import numpy as np
import math
import sys
import matplotlib.pyplot as plt

import airsim
import rospy, actionlib

import lqr
from drone import Drone
from airsim.client import MultirotorClient
from airsim.types import MultirotorState, Vector3r

from geometry_msgs.msg import TwistStamped, PoseStamped, AccelStamped, PoseStamped


from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from std_msgs.msg import Float32

from airsim_ros_cntrl.msg import (
    TrackObjectAction,
    TrackObjectFeedback,
    TrackObjectResult,
    TrackObjectGoal,
    Multirotor
)

from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput


# Calculate vector1 - vector2
def calc_distance(vector1: Vector3r, vector2: Vector3r):
    """
    Calculates the distance between two vectors.

    Args:
        vector1 (Vector3r): [description]
        vector2 (Vector3r): [description]

    Returns:
        Vector3r: Output vector
    """
    output = airsim.Vector3r()

    output.x_val = vector1.x_val - vector2.x_val
    output.y_val = vector1.y_val - vector2.y_val
    output.z_val = vector1.z_val - vector2.z_val
    return output


class Agent(Drone):
    """
    Class to handle individual agents of a team.
    An agent is a drone that takes commands to either track an object or move to a certain location.
    Refer to Drone documentation for more info.

    Args:
        swarmName (str): The name of the team this drone is associated with.
        droneName (str): The name of the drone itself.
        sim_client (airsim.MultirotorClient): The client to use to execture commands.
        client_lock (mp.Lock): The lock for the sim_client.
    """

    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: mp.Lock,
    ) -> None:
        """
        Constructs a new Agent Process.

        Args:
            swarmName (str): The name of the swarm this drone is associated with.
            droneName (str): The name of the drone itself.
            sim_client (airsim.MultirotorClient): The client to use to execture commands.
            client_lock (mp.Lock): The lock for the sim_client.
        """
        super().__init__(swarmName, droneName, sim_client, client_lock)

        self.dstep = 20.0

        self.__controller = lqr.LQR()

        self.rpydot = np.zeros((1, 3))
        self.prev_accel_cmd = 0

        self.__target_pose = MultirotorState()
        self.__target_ready = False

    def setup_ros(self) -> None:
        """
        @Override
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Topics:
            /looptime (Float32): The looptime of the drone
            /sensor/local/odom_ned (nav_msgs.Odometry): Odometry of the agent.
            /sensor/global/gps (sensor_msgs.NavSatFix): The GPS location of the agent
            /sensor/local/imu (sensor_msgs.Imu): The acceleration of the agent

        Services:
            /takeoff (airsim_ros_pkgs.Takeoff): Service to takeoff the drone
            /land (airsim_ros_pkgs.Land): Service to land the drone
            /shutdown (std_srvs.SetBool): Service to shutdown process
            /cmd/pos (geometry_msgs.PoseStamped): Command drone to go to position
        
        Actions:
            /track_object (airsim_ros_cntrl.TrackObjectAction): Action to track a target.

        Returns: None
        """
        Drone.setup_ros(self)
        cmd_pos_topic = self.topic_prefix + "/cmd/pos"

        rospy.Subscriber(
            cmd_pos_topic, PoseStamped, callback=self.__cmd_pos_cb, queue_size=10
        )

        self.__track_action_name = self.topic_prefix + "/track_object"

        self.__track_action = actionlib.SimpleActionServer(
            self.__track_action_name,
            TrackObjectAction,
            execute_cb=self.__track_action_cb,
            auto_start=False,
        )
        self.__track_action.start()

    ##
    ##
    ###        ROS CALLBACKS            ###
    #######################################

    def __target_state_cb(self, msg: PoseStamped) -> None:
        """
        Subscriber callback for tracking target position.
        Used in /track_object

        Args:
            msg (geometry_msgs.PoseStamped)

        Returns: None
        """
        with self.flag_lock:
            pos = msg.state.pose.position
            self.__target_pose.kinematics_estimated.position = airsim.Vector3r(
                pos.x, pos.y, pos.z
            )
        
            vel = msg.state.vel
            self.__target_pose.kinematics_estimated.linear_velocity = airsim.Vector3r(
                vel.linear.x, vel.linear.y, vel.linear.z
            )   
            self.__target_pose.kinematics_estimated.angular_velocity = airsim.Vector3r(
                vel.angular.x, vel.angular.y, vel.angular.z
            )

            acc = msg.state.acc
            self.__target_pose.kinematics_estimated.linear_acceleration = airsim.Vector3r(
                acc.linear.x, acc.linear.y, acc.linear.z
            )
            self.__target_pose.kinematics_estimated.angular_acceleration = airsim.Vector3r(
                acc.angular.x, acc.angular.y, acc.angular.z
            )      
  
            self.__target_ready = True



    def __track_action_cb(self, goal: TrackObjectGoal) -> TrackObjectResult:
        """
        Callback to track a target.
        Subscribes to the position, velocity and acceleration,
        then tracks the object using LQR to control with minimum snap.

        Args:
            goal (TrackObjectGoal): The object to track

        Topic:
            /track_object
        
        Feedback (TrackObjectFeedback): Continuous feedback about how tracking is going

        Returns (TrackObjectResult): The Result after tracking
        """
        start_time = time.time()

        success = True
        target_topic = self.swarm_name + "/" + goal.object_name + "/"

        target_state_sub = rospy.Subscriber(
            target_topic + "multirotor",
            Multirotor,
            callback=self.__target_state_cb,
            queue_size=10,
        )

        while self.__target_ready == False and time.time() - start_time < goal.timeout:
            time.sleep(0.05)

        target_pose = self.__target_pose.kinematics_estimated

        feedback_vector = calc_distance(
            target_pose.position, self.state.kinematics_estimated.position
        )
        feedback_magnitude = airsim.Vector3r.distance_to(
            self.state.kinematics_estimated.position, target_pose.position
        )

        r = rospy.Rate(self.freq)

        feedback = TrackObjectFeedback()

        print(self.drone_name + " ENTERING WHILE LOOP")

        update_object_location_period = 0.2  # seconds
        prev_object_update_time = 0

        start_pos = self.state.kinematics_estimated.position.to_numpy_array()
        start_time = time.time()

        graphviz = GraphvizOutput()
        graphviz.output_file = self.drone_name + "_pycallgraph.png"
        with PyCallGraph(output=graphviz):

            while time.time() - start_time < goal.timeout:
                if self.__track_action.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self.__track_action_name)
                    self.__track_action.set_preempted()
                    success = False
                    break

                pos = self.state.kinematics_estimated.position.to_numpy_array()

                if time.time() - prev_object_update_time > update_object_location_period:
                    target_pose = self.__target_pose.kinematics_estimated

                    bias = np.array(
                        [
                            target_pose.linear_velocity.x_val
                            * update_object_location_period
                            + 0.5
                            * update_object_location_period ** 2
                            * target_pose.linear_acceleration.x_val,
                            target_pose.linear_velocity.y_val
                            * update_object_location_period
                            + 0.5
                            * update_object_location_period ** 2
                            * target_pose.linear_acceleration.y_val,
                            target_pose.linear_velocity.z_val
                            * update_object_location_period
                            + 0.5
                            * update_object_location_period ** 2
                            * target_pose.linear_acceleration.z_val,
                        ]
                    )

                    #bias *= 10
                    bias *= 0

                    p = start_pos#pos
                    
                    v = np.zeros(3)#state.kinematics_estimated.linear_velocity.to_numpy_array()
                    a = np.zeros(3)#state.kinematics_estimated.linear_acceleration.to_numpy_array()
                    j = np.zeros(3)
                    ic = np.array([v, a, j]).T


                    pt = target_pose.position.to_numpy_array() + goal.offset + bias

                    vt = target_pose.linear_velocity.to_numpy_array()
                    at = target_pose.linear_acceleration.to_numpy_array()
                    jt = np.zeros(3)
                    fc = np.array([vt, at, jt]).T

                    #waypoints = np.array([start_pos, pt).T
                    waypoints = np.array([p, pt]).T

                    self.__controller.set_goals(waypoints, ic, fc)

                    prev_object_update_time = time.time()


                #self.moveByLQR(time.time()-prev_object_update_time+0.5, state)
                self.moveByLQR(time.time() - start_time, self.state)

                feedback_vector = airsim.Vector3r(pt[0], pt[1], pt[2])
                feedback_vector = calc_distance(
                    feedback_vector, self.state.kinematics_estimated.position
                )

                feedback.dist = []
                feedback.dist.append(feedback_vector.x_val)
                feedback.dist.append(feedback_vector.y_val)
                feedback.dist.append(feedback_vector.z_val)
                feedback.dist_mag = feedback_vector.get_length()

                self.__track_action.publish_feedback(feedback)

                r.sleep()

        self.__target_ready = False
        target_state_sub.unregister()
        
        with self.client_lock:
            self.client.hoverAsync(self.drone_name)

        if success:
            result = feedback
            self.__track_action.set_succeeded(result)

    def __cmd_pos_cb(self, msg: PoseStamped) -> None:
        """
        Callback for the /cmd/pos ROS topic
        
        Args:
            msg (geometry_msgs.PoseStamped)
        """

        with self.flag_lock:
            self.cmd = msg

    def __moveToPosition(self, cmd: PoseStamped) -> bool:
        """
        Utilize AirSim's Python API to move the drone.

        Args:
            cmd: The command to execute
        
        Returns (bool): True if successful, false otherwise
        """

        with self.flag_lock:
            assert type(cmd) == PoseStamped, "movetoposition cmd must be posestamped"

            pos = self.state.kinematics_estimated.position
            orien = self.state.kinematics_estimated.orientation
            q = [orien.w_val, orien.x_val, orien.y_val, orien.z_val]
            (_, _, yaw) = lqr.LQR.quat2rpy(q)

            if cmd.header.frame_id == "local":
                # ASSUMING 0 PITCH AND ROLL
                x = (
                    cmd.pose.position.x * math.cos(yaw)
                    - cmd.pose.position.y * math.sin(yaw)
                    + pos.x_val
                )
                y = (
                    cmd.pose.position.x * math.sin(yaw)
                    + cmd.pose.position.y * math.cos(yaw)
                    + pos.y_val
                )
                z = cmd.pose.position.z + pos.z_val

            elif cmd.header.frame_id == "global":
                x = cmd.pose.position.x
                y = cmd.pose.position.y
                z = cmd.pose.position.z

            else:
                print("DRONE " + self.drone_name + " VEL cmd UNRECOGNIZED FRAME")
                return False

            p0 = [x, y, z]

            waypoints = np.array([[pos.x, pos.y, pos.z], np.array(p0)])

            error = np.linalg.norm(
                np.array(p0) - self.state.kinematics_estimated.position.to_numpy_array()
            )

            timeout = 10
            begin_time = time.time()

            r = rospy.Rate(self.freq)

            self.__controller.set_goals(waypoints, np.zeros(3), np.zeros(3))

            while time.time() - begin_time < timeout and error > self.dstep / 2:

                self.moveByLQR(time.time() - begin_time, self.state)

                error = np.linalg.norm(
                    np.array(p0) - self.state.kinematics_estimated.position.to_numpy_array()
                )

                r.sleep()
        return True

    ##
    ##
    ###        LQR IMPLEMENATION        ###
    #######################################

    def moveByLQR(self, t: float, state: MultirotorState) -> None:
        """
        Moves the agent via LQR.

        Args:
            t (float): Time elapsed since beginning
            state (MultirotorState): Current Multirotor State
        """
        x0, u0, u = self.__controller.computeControl(t, state, self.prev_accel_cmd)

        for i in range(0, 3):
            if abs(u[i, 0]) > 2:
                print(
                    "WARNING -> RATE "
                    + str(i)
                    + " FOR "
                    + self.drone_name
                    + " GREATER THAN MAX RATE "
                    + str(u[i, 0])
                )

            u[i, 0] = max(-3, u[i, 0])
            u[i, 0] = min(3, u[i, 0])

        # self.rpydot = np.append(self.rpydot, u0[0:3].T, axis=0)

        if u[3, 0] > 1.0:
            print(
                "WARNING -> THROTTLE FOR "
                + self.drone_name
                + " OUT OF BOUNDS "
                + str(u[3, 0])
            )
            u[3, 0] = 1.0

        roll_rate = u[0, 0]
        pitch_rate = u[1, 0]
        yaw_rate = u[2, 0]
        throttle = u[3, 0]

        accel = self.__controller.thrust2world(state, throttle)

        self.prev_acceleration_cmd = accel[2]
        # self.acceleration_cmds = np.append(self.acceleration_cmds, [accel], axis=0)

        self.reference = x0.T

        with self.client_lock:
            self.client.moveByAngleRatesThrottleAsync(
                roll_rate, pitch_rate, yaw_rate, throttle, 0.5, self.drone_name
            )

