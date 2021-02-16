#! /usr/bin/python3

import multiprocessing as mp
import time
from math import cos, pi, sin
from typing import List, Tuple
from airsim.client import MultirotorClient
from airsim.types import MultirotorState
import rospy, os

from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose, Point, Quaternion, Twist, Vector3, AccelStamped, Accel
from std_msgs.msg import Header
from std_msgs.msg import Float32

class Target(mp.Process):
    def __init__(self, swarmName, droneName, sim_client, client_lock, path=[], path_type=""):
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
        mp.Process.__init__(self)

        self.__swarm_name = swarmName
        self.__drone_name = droneName

        self.__shutdown = False
        self.__finished = False

        self.__client_lock = client_lock
        self.__flag_lock = mp.Lock()

        self.freq = 20

        with self.__client_lock:
            self.__client = sim_client
            self.__client.confirmConnection()
            self.__client.enableApiControl(True, vehicle_name=self.__drone_name)
            self.__client.armDisarm(True, vehicle_name=self.__drone_name)
            self.__path_future = self.__client.takeoffAsync(vehicle_name=self.__drone_name)

        self.__path = path if path_type == "" else self.generate_path(path_type)
        self.__path_index = 0

        self.__vehicle_state = self.get_state()

    def __setup_ros(self):
        rospy.init_node(self.__drone_name)

        topic_prefix = "/" + self.__swarm_name + "/" + self.__drone_name
        vel_topic  = topic_prefix + "/vel"
        loop_time_topic = topic_prefix + "/looptime"
        pos_topic = topic_prefix + "/pos"
        acc_topic = topic_prefix + "/acc"

        wait_service_name       = topic_prefix + "/wait"
        shutdown_service_name   = topic_prefix + "/shutdown"

        rospy.init_node(self.__drone_name)
        rospy.on_shutdown(self.shutdown)

        self.__pos_pub = rospy.Publisher(pos_topic, PoseStamped, queue_size=10)
        self.__vel_pub = rospy.Publisher(vel_topic, TwistStamped, queue_size=10)
        self.__acc_pub = rospy.Publisher(acc_topic, AccelStamped, queue_size=10)        
        self.__looptime_pub = rospy.Publisher(loop_time_topic, Float32, queue_size=10)

        rospy.Service(wait_service_name, SetBool, self.__handle_wait) 
        rospy.Service(shutdown_service_name, SetBool, self.__handle_shutdown)

    def __handle_wait(self, req):
        """
        Callback for the /wait rosservice. Waits for maximum of self.__pos_cmd_timeout

        @param req (StdBoolRequest) Currently unused\n
        @return (StdBoolResponse) True if command completed. False if timeout
        """

        with self.__flag_lock:
            if self.__finished:
                return SetBoolResponse(True, "")

            while self.__finished == False and time.time() - self.__time_start <= self.__wait_timeout:
                time.sleep(0.05)

            if self.__finished:
                return SetBoolResponse(True, "")
            else:
                return SetBoolResponse(False, "")

    def __handle_shutdown(self, req):
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

    def shutdown(self):
        """
        Handle improper rospy shutdown. Uses Python API to disarm drone
        """
        with self.__flag_lock:
            self.__shutdown = True
            self.__finished = True

        print(self.__drone_name + " QUITTING")


    def get_client(self):
        return self.__client

    def get_name(self):
        return self.__drone_name

    def get_state(self):
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
                    self.__vehicle_state = self.__client.getMultirotorState(vehicle_name=self.__drone_name)
                    break

                except Exception as e:
                    error = e
                    error_count += 1

            if error_count == max_errors:
                print(self.__drone_name + " Error from getMultirotorState API call: {0}" .format(error.message))

            for _ in range(0, max_errors):
                try:
                    pose = self.__client.simGetObjectPose(object_name=self.__drone_name)
                    break

                except Exception as e:
                    error = e
                    error_count += 1

            if error_count == max_errors:
                print(self.__drone_name + " Error from getMultirotorState API call: {0}" .format(error.message))

        return pose, self.__vehicle_state

    def generate_path(self, path_type, radius=5, height=-5):
        paths = {
            "circle": 12,
            "triangle": 3,
            "square": 4,
            "line": 2
        }
        points = paths[path_type]
        center = self.get_state().kinematics_estimated.position
        path = []
        for i in range(points):
            x = radius * cos(pi / points * i * 2) + center.x_val
            y = radius * sin(pi / points * i * 2) + center.y_val
            path.append((x, y, height))
        return path

    def run(self):
        self.__setup_ros()

        time.sleep(8)

        rate = rospy.Rate(self.freq)
        
        prev_time = time.time()

        with self.__client_lock:
            self.__path_future = self.__client.moveToPositionAsync(*self.__path[self.__path_index], 2, 20, vehicle_name=self.__drone_name)
        self.__path_index += 1
        if self.__path_index == len(self.__path):
            self.__path_index = 0

        while not rospy.is_shutdown():
            with self.__flag_lock:
                if self.__shutdown == True:
                    break
            
            '''
            if self.__path_future._result is not None:
                with self.__client_lock:
                    self.__path_future = self.__client.moveToPositionAsync(self.__path[self.__path_index][0], self.__path[self.__path_index][1], self.__path[self.__path_index][2], 5)
                self.__path_index += 1
                if self.__path_index == len(self.__path):
                    self.__path_index = 0
            '''

            pose, state = self.get_state()
            state = state.kinematics_estimated

            if self.__pos_pub:
                point = Point(pose.position.x_val, pose.position.y_val, pose.position.z_val)
                quat = Quaternion(pose.orientation.x_val, pose.orientation.y_val, pose.orientation.z_val, pose.orientation.w_val)
                pose = Pose(point, quat)
                header = Header()
                header.stamp = rospy.Time.now()

                msg = PoseStamped(header, pose)
                self.__pos_pub.publish(msg)

            if self.__vel_pub:
                linear = Vector3(state.linear_velocity.x_val, state.linear_velocity.y_val, state.linear_velocity.z_val)
                angular = Vector3(state.angular_velocity.x_val, state.angular_velocity.y_val, state.angular_velocity.z_val)
                twist = Twist(linear, angular)
                header = Header()
                header.stamp = rospy.Time.now()
                msg = TwistStamped(header, twist)
                self.__vel_pub.publish(msg)

            if self.__acc_pub:
                linear = Vector3(state.linear_acceleration.x_val, state.linear_acceleration.y_val, state.linear_acceleration.z_val)
                angular = Vector3(state.angular_acceleration.x_val, state.angular_acceleration.y_val, state.angular_acceleration.z_val)
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

        #self.__path_future.join()

        print(self.__drone_name + " QUITTING")