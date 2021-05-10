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

from geometry_msgs.msg import TwistStamped, AccelStamped, Pose, PoseStamped,  Twist, Quaternion, Point, Vector3


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

from airsim_ros_pkgs.msg import GimbalAngleEulerCmd, GimbalAngleQuatCmd

from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput


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
        self.print_debug = False

        print("NEW AGENT: " + self.drone_name)


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

        target_topic = self.swarm_name + "/Target0/"
        target_state_sub = rospy.Subscriber(
            target_topic + "multirotor",
            Multirotor,
            callback=self.__target_state_cb,
            queue_size=10,
        )

        self.__gimbal_pub = rospy.Publisher("/airsim_node/gimbal_angle_quat_cmd", GimbalAngleQuatCmd, queue_size=1)
        self.__desired_pose_pub = rospy.Publisher(self.swarm_name + "/" + self.drone_name + "/lqr/desired_pose", Pose, queue_size=1)
        self.__desired_vel_pub = rospy.Publisher(self.swarm_name + "/" + self.drone_name + "/lqr/desired_vel", Twist, queue_size=1)

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

        #r = rospy.Rate(self.freq)
        sleeper = rospy.Rate(self.freq)


        feedback = TrackObjectFeedback()
        feedback_pos_save = np.empty((0,3))
        feedback_vel_save = np.empty((0,3))


        print(self.drone_name + " ENTERING WHILE LOOP")

        update_object_location_period = 1.0  # seconds
        prev_object_update_time = 0

        start_pos = self.state.kinematics_estimated.position.to_numpy_array()
        start_time = time.time()

        graphviz = GraphvizOutput()
        graphviz.output_file = self.drone_name + "_pycallgraph.png"


        prev_time = time.time()
        
        
        with PyCallGraph(output=graphviz):

            while time.time() - start_time < goal.timeout:
                if self.__track_action.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self.__track_action_name)
                    self.__track_action.set_preempted()
                    success = False
                    break

                pos = self.state.kinematics_estimated.position.to_numpy_array()
                target_pose = self.__target_pose.kinematics_estimated
                tp = target_pose.position.to_numpy_array()

                if time.time() - prev_object_update_time > update_object_location_period:

                    target_vel = target_pose.linear_velocity.to_numpy_array()
                    target_acc = target_pose.linear_acceleration.to_numpy_array()

                    bias = target_vel*update_object_location_period + 0.5*target_acc*update_object_location_period**2

                    p = pos
                    
                    v = self.state.kinematics_estimated.linear_velocity.to_numpy_array()
                    a = self.state.kinematics_estimated.linear_acceleration.to_numpy_array()
                    j = np.zeros(3)
                    ic = np.array([v, a, j])


                    pt = tp + goal.offset + bias

                    at = target_acc
                    vt = target_vel + target_acc*update_object_location_period
                    jt = np.zeros(3)
                    fc = np.array([vt,at,jt])

                    #waypoints = np.array([start_pos, pt]).T
                    waypoints = np.array([p, pt]).T

                    d = np.linalg.norm((p-(tp+goal.offset)))

                    #spd_gain = np.interp(d, [0, 30], [0, 0.1])
                    spd_gain = 0.125

                    if d < 0.25:
                        spd_gain = 0.0

                    avg_spd = np.linalg.norm(target_vel) + spd_gain*d
                    avg_spd = np.minimum(avg_spd, 2.0)
                    #avg_spd = np.maximum(avg_spd, 1)

                    self.__controller.set_goals(waypoints, ic, fc, avg_spd)

                    prev_object_update_time = time.time()


                self.moveByLQR(prev_object_update_time, self.state)

                feedback_vector = airsim.Vector3r(*(tp+goal.offset)) - self.state.kinematics_estimated.position

                feedback.dist = []
                feedback.dist.append(feedback_vector.x_val)
                feedback.dist.append(feedback_vector.y_val)
                feedback.dist.append(feedback_vector.z_val)
                feedback.dist_mag = feedback_vector.get_length()


                '''
                feedback_dir = feedback_vector/feedback_vector.get_length()

                normpos = self.state.kinematics_estimated.position/self.state.kinematics_estimated.position.get_length()
                normtar = target_pose.position/target_pose.position.get_length()

                gimbal_orien = target_pose.position.cross(self.state.kinematics_estimated.position).to_Quaternionr()
                gimbal_orien.w_val = math.sqrt((target_pose.position.get_length()**2)*(self.state.kinematics_estimated.position.get_length()**2)) +  math.acos(normtar.dot(normpos))
                gimbal_orien = gimbal_orien / gimbal_orien.get_length()

                (gp, gr, gy) = airsim.to_eularian_angles(gimbal_orien)
                (p,r,y) = airsim.to_eularian_angles(self.state.kinematics_estimated.orientation)


                gimbal_msg = GimbalAngleQuatCmd()
                #gimbal_msg.orientation = Quaternion(*gimbal_orien.to_numpy_array())
                gimbal_msg.camera_name = "cam0"
                gimbal_msg.vehicle_name = self.drone_name

                #self.__gimbal_pub.publish(gimbal_msg)
                '''

                self.__track_action.publish_feedback(feedback)


                np.append(feedback_pos_save, feedback_vector.to_numpy_array())
                sleeper.sleep()

                #print(self.drone_name + " track time: " + str(time.time()-prev_time))
                prev_time = time.time()

                
        self.cmd = None


        self.__target_ready = False
        target_state_sub.unregister()


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


    ##
    ##
    ###        LQR IMPLEMENATION        ###
    #######################################

    def moveByLQR(self, t0: float, state: MultirotorState) -> None:
        """
        Moves the agent via LQR.

        Args:
            t (float): Time elapsed since beginning
            state (MultirotorState): Current Multirotor State
        """
        x0, u = self.__controller.computeControl(t0, state, self.prev_accel_cmd, self.drone_name)

        for i in range(0, 3):
            if abs(u[i, 0]) > 2:
                
                if self.print_debug:
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

        if u[3, 0] > 1.0:
            if self.print_debug:
                print(
                    "WARNING -> THROTTLE FOR "
                    + self.drone_name
                    + " OUT OF BOUNDS "
                    + str(u[3, 0])
                )

            u[3, 0] = 1.0

        roll_rate  = u[0, 0]
        pitch_rate = u[1, 0]
        yaw_rate   = u[2, 0]
        throttle   = u[3, 0]

        accel = self.__controller.thrust2world(state, throttle)

        self.prev_acceleration_cmd = accel[2]

        throttle_rates_cmd = Twist()
        throttle_rates_cmd.linear.z = throttle
        throttle_rates_cmd.angular.x = roll_rate
        throttle_rates_cmd.angular.y = pitch_rate
        throttle_rates_cmd.angular.z = yaw_rate

        self.throttle_rates_cmd_pub.publish(throttle_rates_cmd)

        
        (pDes, qDes, vDes) = lqr.LQR.get_state(x0)
        desired_pose_msg = Pose()
        desired_pose_msg.position = Point(*pDes)
        desired_pose_msg.orientation = Quaternion(w=qDes[0], x=qDes[1], y=qDes[2], z=qDes[3])
        self.__desired_pose_pub.publish(desired_pose_msg)

        desired_vel_msg = Twist()
        desired_vel_msg.linear = Vector3(*vDes)
        desired_vel_msg.angular = Vector3(*u[0:3,0])
        self.__desired_vel_pub.publish(desired_vel_msg)