#! /usr/bin/python

import time
import multiprocessing as mp
import math

import airsim
import rospy

from geometry_msgs.msg import TwistStamped, PoseStamped
from airsim_ros_pkgs.srv import Takeoff, TakeoffResponse, Land, LandResponse

from std_srvs.srv import SetBool, SetBoolResponse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu


def makeVelCmd(frame="local", lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
    vel_cmd = TwistStamped()
    vel_cmd.header.frame_id = frame
    vel_cmd.header.stamp = rospy.Time.now()

    vel_cmd.twist.linear.x = lx
    vel_cmd.twist.linear.y = ly
    vel_cmd.twist.linear.z = lz
    vel_cmd.twist.angular.x = ax
    vel_cmd.twist.angular.y = ay
    vel_cmd.twist.angular.z = az

    return vel_cmd

class Drone(mp.Process):
    def __init__(self, swarmName, droneName, sim_client, client_lock):
        mp.Process.__init__(self)

        self.__swarm_name = swarmName
        self.__drone_name = droneName

        self.__client_lock = client_lock

        self.__command_type = None
        self.__cmd = None

        with self.__client_lock:
            self.__client = sim_client
            self.__client.confirmConnection()
            self.__client.enableApiControl(True, vehicle_name=self.__drone_name)
            self.__client.armDisarm(True, vehicle_name=self.__drone_name)

        self.__vehicle_state = self.get_state()
        
        self.__shutdown = False
        self.__finished = False
        self.__time_start = time.time()

        self.__pos_cmd_timeout = 5.0
        self.__vel_cmd_timeout = 0.1
        self.__service_timeout = 5.0

    def __setup_ros(self):
        topic_prefix = "/" + self.__swarm_name + "/" + self.__drone_name
        cmd_vel_topic  = topic_prefix + "/cmd/vel"

        cmd_pos_topic = topic_prefix + "/cmd/pos"

        odom_topic  = topic_prefix + "/sensor/local/odom_ned"
        gps_topic   = topic_prefix + "/sensor/global/gps"
        imu_topic   = topic_prefix + "/sensor/local/imu"

        takeoff_service_name    = topic_prefix + "/takeoff"
        land_service_name       = topic_prefix + "/land"
        wait_service_name       = topic_prefix + "/wait"


        rospy.init_node(self.__drone_name)

        self.__odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        self.__gps_pub = rospy.Publisher(gps_topic, NavSatFix, queue_size=10)
        self.__imu_topic = rospy.Publisher(imu_topic, Imu, queue_size=10)
        
        rospy.Subscriber(cmd_vel_topic, TwistStamped, callback=self.__cmd_vel_cb, queue_size=10)

        # rospy.Subscriber(cmd_pos_local_topic, )  TODO: MAKE A ROS ACTION SERVER FOR POSITION AND PATHS
        # TODO : MAKE ROS ACTION SERVER FOR POSITION CMD
        # TODO : MAKE ROS ACTION SERVER FOR PATH CMD


        self.__takeoff_service  = rospy.Service(takeoff_service_name, Takeoff, self.__handle_takeoff)
        self.__land_service     = rospy.Service(land_service_name, Land, self.__handle_land)
        self.__wait_service     = rospy.Service(wait_service_name, SetBool, self.__handle_wait) 

        # TODO : MAKE ROS SERVICE TO TOGGLE EACH ROS TOPIC
        # TODO : MAKE ROS SERVICE FOR LANDING
        # TODO : MAKE ROS SERVICE FOR TAKEOFF
        # TODO : MAKE ROS SERVICE FOR ARM/DISARM
        # TODO : MAKE ROS SERVICE TO GET STATE
        # TODO : MAKE ROS SERVICE TO WAIT FOR LAST TAKE TO COMPLETE
        # TODO : MAKE ROS SERVICE TO SHUTDOWN DRONE
        # TODO : MAKE ROS SERVICE TO ADJUST TIMEOUTS



        
    def get_client(self):
        return self.__client

    def get_name(self):
        return self.__drone_name

    def get_state(self):
        with self.__client_lock:
            self.__vehicle_state = self.__client.getMultirotorState(vehicle_name=self.__drone_name)
        return self.__vehicle_state

    def __cmd_vel_cb(self, msg):
        self.__time_start = time.time()
        self.__finished = False      
        self.__cmd = msg

    def __moveAtVelocity(self):
        if type(self.__cmd) != TwistStamped:
            self.__cmd = None
            self.__finished = True
            return False

        if self.__cmd.header.frame_id == "local":
            (pitch, roll, yaw) = airsim.to_eularian_angles(self.get_state().kinematics_estimated.orientation)
            
            # ASSUMING 0 PITCH AND ROLL
            x = self.__cmd.twist.linear.x*math.cos(yaw) - self.__cmd.twist.linear.y*math.sin(yaw)
            y = self.__cmd.twist.linear.x*math.sin(yaw) + self.__cmd.twist.linear.y*math.cos(yaw)
            z = self.__cmd.twist.linear.z
            yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(self.__cmd.twist.angular.z))
        
        elif self.__cmd.header.frame_id == "global":
            x = self.__cmd.twist.linear.x
            y = self.__cmd.twist.linear.y
            z = self.__cmd.twist.linear.z
            yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(self.__cmd.twist.angular.z))

        else:
            print("DRONE " + self.__drone_name + " VEL cmd UNRECOGNIZED FRAME")
            self.__finished = True
            return False

        with self.__client_lock:
            self.__client.moveByVelocityAsync(x, y, z, duration=self.__vel_cmd_timeout, yaw_mode=yawmode, vehicle_name=self.__drone_name)



    def __handle_takeoff(self, req):
        if self.get_state().landed_state == airsim.LandedState.Flying:
            return TakeoffResponse(True)

        time_start = time.time()
        with self.__client_lock:
            self.__client.takeoffAsync(vehicle_name=self.__drone_name)

        if req.waitOnLastTask == False:
            return TakeoffResponse(False)

        while self.get_state().landed_state != airsim.LandedState.Flying and time.time() - time_start < self.__service_timeout:
            time.sleep(0.05)

        if self.__vehicle_state.landed_state == airsim.LandedState.Flying:
            return TakeoffResponse(True)
        else:
            return TakeoffResponse(False)

    def __handle_land(self, req):
        if self.get_state().landed_state == airsim.LandedState.Landed:
            return LandResponse(True)

        time_start = time.time()
        with self.__client_lock:
            self.__client.landAsync(vehicle_name=self.__drone_name)

        if req.waitOnLastTask == False:
            return LandResponse(False)

        while self.get_state().landed_state != airsim.LandedState.Landed and time.time() - time_start < self.__service_timeout:
            time.sleep(0.05)

        if self.__vehicle_state.landed_state == airsim.LandedState.Landed:
            return LandResponse(True)
        else:
            return LandResponse(False)        

    def __handle_wait(self, req):
        if self.__finished:
            return SetBoolResponse(True)

        while self.__finished == False and time.time() - self.__time_start <= self.__pos_cmd_timeout:
            time.sleep(0.05)

        if self.__finished:
            return SetBoolResponse(True)
        else:
            return SetBoolResponse(False)




    def run(self):
        self.__setup_ros()

        rate = rospy.Rate(100)
        while self.__shutdown == False and not rospy.is_shutdown():

            if(type(self.__cmd) == TwistStamped):
                self.__moveAtVelocity()

            rate.sleep()


