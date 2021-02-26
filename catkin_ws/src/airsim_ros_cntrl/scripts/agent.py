#! /usr/bin/python3

import time
import multiprocessing as mp
import numpy as np
import math
import sys, os
import matplotlib.pyplot as plt

import airsim, minimum_snap
import rospy, actionlib

import lqr
from drone import Drone
from airsim.client import MultirotorClient
from airsim.types import MultirotorState

from geometry_msgs.msg import TwistStamped, PoseStamped, AccelStamped, PoseStamped
from airsim_ros_pkgs.srv import Takeoff, TakeoffResponse, Land, LandResponse

from std_srvs.srv import SetBool, SetBoolResponse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from std_msgs.msg import Float32

from airsim_ros_cntrl.msg import (
    TrackObjectAction,
    TrackObjectFeedback,
    TrackObjectResult,
    TrackObjectGoal,
)
from airsim_ros_cntrl.msg import (
    MoveToLocationAction,
    MoveToLocationFeedback,
    MoveToLocationResult,
    MoveToLocationGoal,
)


from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput

# Calculate vector1 - vector2
def calc_distance(vector1, vector2):
    output = airsim.Vector3r()

    output.x_val = vector1.x_val - vector2.x_val
    output.y_val = vector1.y_val - vector2.y_val
    output.z_val = vector1.z_val - vector2.z_val
    return output


class Agent(Drone):
    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: mp.Lock,
    ) -> None:
        """
        Initialize a Drone process, spinup the ROS node, and setup topics/services/action servers

        All topic names are set up as /swarm_name/drone_name/major_cmd/minor_cmd. Eg. commanding the velocity
        of drone1 in swarm1 would be: /swarm1/drones1/cmd/vel

        @param swarmName (string) Name of the UAV swarm
        ---
        @param droneName (string) Name of the UAV
        ---
        @param sim_client (airsim.MultirotorClient) Airsim Python Client
        ---
        """

        super().__init__(swarmName, droneName, sim_client, client_lock)

        self.__cmd = None

        self.dstep = 20.0

        self.__controller = lqr.LQR()

        self.rpydot = np.zeros((1, 3))

        self.__target_pose = MultirotorState()
        self.__target_ready = False

    def __setup_ros(self) -> None:
        super().__setup_ros()
        cmd_pos_topic = self.__topic_prefix + "/cmd/pos"

        odom_topic = self.__topic_prefix + "/sensor/local/odom_ned"
        gps_topic = self.__topic_prefix + "/sensor/global/gps"
        imu_topic = self.__topic_prefix + "/sensor/local/imu"

        rospy.init_node(self.__drone_name)

        self.__odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        self.__gps_pub = rospy.Publisher(gps_topic, NavSatFix, queue_size=10)
        self.__imu_topic = rospy.Publisher(imu_topic, Imu, queue_size=10)

        rospy.Subscriber(
            cmd_pos_topic, PoseStamped, callback=self.__cmd_pos_cb, queue_size=10
        )

        self.__track_action_name = self.__topic_prefix + "/track_object"
        self.__move_to_location_action_name = self.__topic_prefix + "/move_to_location"

        self.__track_action = actionlib.SimpleActionServer(
            self.__track_action_name,
            TrackObjectAction,
            execute_cb=self.__track_action_cb,
            auto_start=False,
        )
        self.__track_action.start()

        self.__move_to_location_action = actionlib.SimpleActionServer(
            self.__move_to_location_action_name,
            MoveToLocationAction,
            execute_cb=self.__move_to_location_action_cb,
            auto_start=False,
        )
        self.__move_to_location_action.start()

    ##
    ##
    ###        ROS CALLBACKS            ###
    #######################################

    def __target_pose_cb(self, msg: PoseStamped) -> None:
        """
        Callback for the /cmd/pos ROS topic

        @param msg (geometry_msgs.PoseStamped)
        """

        with self.__flag_lock:
            pos = msg.pose.position
            self.__target_pose.kinematics_estimated.position = airsim.Vector3r(
                pos.x, pos.y, pos.z
            )
            self.__target_ready = True

    def __target_vel_cb(self, msg: TwistStamped) -> None:
        with self.__flag_lock:
            vel = msg.twist.linear
            self.__target_pose.kinematics_estimated.linear_velocity = airsim.Vector3r(
                vel.x, vel.y, vel.z
            )
            self.__target_ready = True

    def __target_acc_cb(self, msg: AccelStamped) -> None:
        with self.__flag_lock:
            accel = msg.accel.linear
            self.__target_pose.kinematics_estimated.linear_acceleration = (
                airsim.Vector3r(accel.x, accel.y, accel.z)
            )
            self.__target_ready = True

    def __track_action_cb(self, goal):
        start_time = time.time()

        success = True

        state = self.get_state()

        target_topic = self.__swarm_name + "/" + goal.object_name + "/"

        target_pos_sub = rospy.Subscriber(
            target_topic + "pos",
            PoseStamped,
            callback=self.__target_pose_cb,
            queue_size=10,
        )
        target_vel_sub = rospy.Subscriber(
            target_topic + "vel",
            TwistStamped,
            callback=self.__target_vel_cb,
            queue_size=10,
        )
        target_acc_sub = rospy.Subscriber(
            target_topic + "acc",
            AccelStamped,
            callback=self.__target_acc_cb,
            queue_size=10,
        )

        while self.__target_ready == False and time.time() - start_time < goal.timeout:
            time.sleep(0.05)

        target_pose = self.__target_pose.kinematics_estimated

        feedback_vector = calc_distance(
            target_pose.position, state.kinematics_estimated.position
        )
        feedback_magnitude = airsim.Vector3r.distance_to(
            state.kinematics_estimated.position, target_pose.position
        )

        r = rospy.Rate(20)

        feedback = TrackObjectFeedback()

        print(self.__drone_name + " ENTERING WHILE LOOP")

        update_object_location_period = 0.1  # seconds
        prev_object_update_time = 0

        start_pos = state.kinematics_estimated.position.to_numpy_array()
        start_time = time.time()

        while time.time() - start_time < goal.timeout:
            if self.__track_action.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.__track_action_name)
                self.__track_action.set_preempted()
                success = False
                break

            pos = state.kinematics_estimated.position.to_numpy_array()

            if time.time() - prev_object_update_time > update_object_location_period:
                target_pose = self.__target_pose.kinematics_estimated
                prev_object_update_time = time.time()

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

                bias *= 10

                x = target_pose.position.x_val + goal.offset[0] + bias[0]
                y = target_pose.position.y_val + goal.offset[1] + bias[1]
                z = target_pose.position.z_val + goal.offset[2] + bias[2]

                waypoints = np.array([start_pos, [x, y, z]]).T
                self.__controller.set_goals(waypoints)

            # self.moveByLQR(time.time()-prev_object_update_time, state)
            self.moveByLQR(time.time() - start_time, state)

            state = self.get_state()

            feedback_vector = airsim.Vector3r(x, y, z)
            feedback_vector = calc_distance(
                feedback_vector, state.kinematics_estimated.position
            )

            feedback_magnitude = airsim.Vector3r.distance_to(
                feedback_vector, state.kinematics_estimated.position
            )

            feedback.dist = []
            feedback.dist.append(feedback_vector.x_val)
            feedback.dist.append(feedback_vector.y_val)
            feedback.dist.append(feedback_vector.z_val)
            feedback.dist_mag = feedback_magnitude

            self.__track_action.publish_feedback(feedback)

            r.sleep()

        self.__target_ready = False
        target_pos_sub.unregister()
        target_vel_sub.unregister()
        target_acc_sub.unregister()

        with self.__client_lock:
            self.__client.hoverAsync(self.__drone_name)

        if success:
            result = feedback
            self.__track_action.set_succeeded(result)

    def __cmd_pos_cb(self, msg: PoseStamped):
        """
        Callback for the /cmd/pos ROS topic
        @param msg (geometry_msgs.PoseStamped)
        """

        with self.__flag_lock:
            self.__time_start = time.time()
            self.__cmd = msg

    def __moveToPosition(self, cmd):
        """
        Utilize AirSim's Python API to move the drone
        """

        with self.__flag_lock:
            assert type(cmd) == PoseStamped, "movetoposition cmd must be posestamped"

            state = self.get_state()
            pos = state.kinematics_estimated.position
            orien = state.kinematics_estimated.orientation
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
                print("DRONE " + self.__drone_name + " VEL cmd UNRECOGNIZED FRAME")
                return False

            p0 = [x, y, z]

            waypoints = np.array([[pos.x, pos.y, pos.z], np.array(p0)])

            error = np.linalg.norm(
                np.array(p0) - state.kinematics_estimated.position.to_numpy_array()
            )

            timeout = 10
            begin_time = time.time()

            r = rospy.Rate(20)

            self.__controller.set_goals(waypoints)

            while time.time() - begin_time < timeout and error > self.dstep / 2:

                self.moveByLQR(time.time() - begin_time, state)

                state = self.get_state()
                error = np.linalg.norm(
                    np.array(p0) - state.kinematics_estimated.position.to_numpy_array()
                )

                r.sleep()

    ##
    ##
    ###        LQR IMPLEMENATION        ###
    #######################################

    def moveByLQR(self, t, state):
        x0, u0, u = self.__controller.computeControl(t, state, self.prev_accel_cmd)

        for i in range(0, 3):
            if abs(u[i, 0]) > 2:
                print(
                    "WARNING -> RATE "
                    + str(i)
                    + " FOR "
                    + self.__drone_name
                    + " GREATER THAN MAX RATE "
                    + str(u[i, 0])
                )

            u[i, 0] = max(-3, u[i, 0])
            u[i, 0] = min(3, u[i, 0])

        # self.rpydot = np.append(self.rpydot, u0[0:3].T, axis=0)

        if u[3, 0] > 1.0:
            print(
                "WARNING -> THROTTLE FOR "
                + self.__drone_name
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

        with self.__client_lock:
            self.__client.moveByAngleRatesThrottleAsync(
                roll_rate, pitch_rate, yaw_rate, throttle, 0.5, self.__drone_name
            )

    def createGraphs(self, rate: rospy.Rate):
        times = np.zeros((1, 1))

        # Setup timing variables
        begin_time = time.time()

        with PyCallGraph(output=GraphvizOutput()):

            font = {"weight": "bold", "size": 12}

            plt.rc("font", **font)

            plt.grid()
            plots = [
                plt.subplot(221),
                plt.subplot(222),
                plt.subplot(223),
                plt.subplot(224),
            ]

            references = np.zeros((1, 10))

            # Main loop
            while not rospy.is_shutdown() and time.time() - begin_time < self.sim_time:
                with self.__flag_lock:
                    if self.__shutdown == True:
                        break

                    state = self.get_state()
                    pos = state.kinematics_estimated.position.to_numpy_array()
                    vel = state.kinematics_estimated.linear_velocity.to_numpy_array()

                    q = state.kinematics_estimated.orientation
                    q = np.array([q.w_val, q.x_val, q.y_val, q.z_val], dtype=np.float32)
                    r, p, y = lqr.LQR.quat2rpy(q)
                    orien = np.array([r, p, y], dtype=np.float32)

                    rpydot = (
                        state.kinematics_estimated.angular_velocity.to_numpy_array()
                    )
                    accel = (
                        state.kinematics_estimated.linear_acceleration.to_numpy_array()
                    )

                    statevector = np.concatenate(
                        (pos, vel, orien, rpydot, accel), axis=0
                    )

                    states = np.append(states, [statevector], axis=0)
                    times = np.append(times, [[time.time() - begin_time]], axis=0)

                    cmd = self.__cmd

                    if type(cmd) == PoseStamped:
                        self.__moveToPosition(cmd)

                    self.__cmd = None

                self.moveByLQR(time.time() - begin_time, state)

                references = np.append(references, self.reference, axis=0)

                rate.sleep()

                # Calculate a publish looptime after sleeping
                if self.__pub_looptime:
                    elapsed_time = time.time() - prev_time
                    prev_time = time.time()

                    msg = Float32(elapsed_time)
                    self.__looptime_pub.publish(msg)
                    # print("%6.3f"% elapsed_time)

        print(
            "Theoretical number of loops: "
            + str(int(math.ceil(self.sim_time * self.freq)))
        )
        print("Actual number of loops: " + str(times.shape[0]))

        plots[0].plot(times, states[:, 0], "b", label="n")
        plots[0].plot(times, states[:, 1], "r", label="e")
        plots[0].plot(times, states[:, 2], "g", label="d")
        plots[0].plot(times, references[:, 0], "--b", label="n0")
        plots[0].plot(times, references[:, 1], "--r", label="e0")
        plots[0].plot(times, references[:, 2], "--g", label="d0")
        # plots[0].title('position', fontsize=12, fontweight='bold')
        # plots[0].xlabel('time', fontsize=12, fontweight='bold')
        # plots[0].ylabel('m', fontsize=12, fontweight='bold')
        plots[0].set(title="position", xlabel="time", ylabel="m")
        plots[0].legend(loc="upper right")

        plots[1].plot(times, states[:, 3], "b", label="vn")
        plots[1].plot(times, states[:, 4], "r", label="ve")
        plots[1].plot(times, states[:, 5], "g", label="vd")
        plots[1].plot(times, references[:, 7], "--b", label="vn0")
        plots[1].plot(times, references[:, 8], "--r", label="ve0")
        plots[1].plot(times, references[:, 9], "--g", label="vd0")
        plots[1].set(xlabel="time", ylabel="m/s")
        plots[1].set_title("ned vecloties")
        plots[1].legend(loc="upper right")

        # plots[2].plot(times, states[:,6], color='blue', label='r')
        # plots[2].plot(times, states[:,7], color='red', label='p')
        # plots[2].plot(times, states[:,8], color='gray', label='y')
        # plots[2].set(xlabel='time', ylabel='rad')
        # plots[2].set_title('orientation')
        # plots[2].legend(loc="upper right")

        plots[2].plot(times, states[:, 9], "b", label="dr")
        plots[2].plot(times, states[:, 10], "r", label="dp")
        plots[2].plot(times, states[:, 11], "g", label="dy")
        plots[2].plot(times, self.rpydot[:, 0], "--b", label="dr0")
        plots[2].plot(times, self.rpydot[:, 1], "--r", label="dp0")
        plots[2].plot(times, self.rpydot[:, 2], "--g", label="dy0")
        plots[2].set(xlabel="time", ylabel="rad/s")
        plots[2].set_title("angular velocity")
        plots[2].legend(loc="upper right")

        plots[3].set_title("angular velocity")
        plots[3].plot(times, states[:, 12], "b", label="an")
        plots[3].plot(times, states[:, 13], "r", label="ae")
        plots[3].plot(times, states[:, 14], "g", label="ad")
        plots[3].plot(times, self.acceleration_cmds[:, 0], "--b", label="an0")
        plots[3].plot(times, self.acceleration_cmds[:, 1], "--r", label="ae0")
        plots[3].plot(times, self.acceleration_cmds[:, 2], "--g", label="ad0")
        plots[3].set(xlabel="time", ylabel="m/s^2")
        plots[3].set_title("linear acceleration")
        plots[3].legend(loc="upper right")

        plt.show()

    def run(self):
        self.__setup_ros()

        self.sim_time = 20
        self.prev_accel_cmd = 0

        rate = rospy.Rate(self.freq)

        prev_time = time.time()

        run_swarm = True

        if run_swarm:
            while not rospy.is_shutdown() or self.__shutdown:
                rate.sleep()

                # Calculate a publish looptime after sleeping
                if self.__pub_looptime:
                    elapsed_time = time.time() - prev_time
                    prev_time = time.time()

                    msg = Float32(elapsed_time)
                    self.__looptime_pub.publish(msg)
                    # print("%6.3f"% elapsed_time)

        else:
            # Move to starting position (0,0,-5)
            with self.__client_lock:
                # self.__client.takeoffAsync(vehicle_name=self.__drone_name).join()
                self.__client.moveToPositionAsync(
                    0, 0, -4, 5, vehicle_name=self.__drone_name
                ).join()
                time.sleep(3)
                print("Reached (0,0,-30), starting motion")

            states = np.zeros((1, 15))
            states[
                0, 0:3
            ] = self.get_state().kinematics_estimated.position.to_numpy_array()

            self.acceleration_cmds = np.zeros((1, 3))

            # Setup initial reference position
            p0 = self.get_state().kinematics_estimated.position.to_numpy_array()
            self.reference = np.concatenate((p0, [1, 0, 0, 0, 0, 0, 0]), axis=0)

            waypoints = np.array([p0, [5, 5, -5], [13, 10, -5], [14, 10, -15]]).T

            self.__controller.set_goals(waypoints)

            self.createGraphs()

        # Wait for last task to finish
        with self.__client_lock:
            self.__client.cancelLastTask(vehicle_name=self.__drone_name)

        # Quit
        print(self.__drone_name + " QUITTING")
        time.sleep(0.5)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])

    client = airsim.MultirotorClient(ip="192.168.1.2")
    lock = mp.Lock()

    drone = Agent("swarm", drone_name, client, lock)
    drone.start()
    drone.join()
