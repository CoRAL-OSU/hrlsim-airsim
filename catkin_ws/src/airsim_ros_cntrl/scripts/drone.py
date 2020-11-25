#! /usr/bin/python

import time
import multiprocessing as mp
import math
import sys

import airsim
import rospy

from geometry_msgs.msg import TwistStamped, PoseStamped
from airsim_ros_pkgs.srv import Takeoff, TakeoffResponse, Land, LandResponse

from std_srvs.srv import SetBool, SetBoolResponse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from std_msgs.msg import Float32


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

        mp.Process.__init__(self)

        self.__swarm_name = swarmName
        self.__drone_name = droneName

        self.__client_lock = client_lock
        self.__flag_lock = mp.Lock()

        self.__command_type = None
        self.__cmd = None

        with self.__client_lock:
            self.__client = sim_client
            self.__client.confirmConnection()
            self.__client.enableApiControl(True, vehicle_name=self.__drone_name)
            self.__client.armDisarm(True, vehicle_name=self.__drone_name)

        self.__vehicle_state = self.get_state()
        
        self.__pub_looptime = True
        self.__shutdown = False
        self.__finished = False
        self.__time_start = time.time()

        self.__pos_cmd_timeout = 5.0
        self.__vel_cmd_timeout = 0.1
        self.__service_timeout = 5.0

    def __setup_ros(self):
        topic_prefix = "/" + self.__swarm_name + "/" + self.__drone_name
        cmd_vel_topic  = topic_prefix + "/cmd/vel"
        loop_time_topic = topic_prefix + "/looptime"
        cmd_pos_topic = topic_prefix + "/cmd/pos"

        odom_topic  = topic_prefix + "/sensor/local/odom_ned"
        gps_topic   = topic_prefix + "/sensor/global/gps"
        imu_topic   = topic_prefix + "/sensor/local/imu"

        takeoff_service_name    = topic_prefix + "/takeoff"
        land_service_name       = topic_prefix + "/land"
        wait_service_name       = topic_prefix + "/wait"
        shutdown_service_name   = topic_prefix + "/shutdown"


        rospy.init_node(self.__drone_name)

        self.__odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        self.__gps_pub = rospy.Publisher(gps_topic, NavSatFix, queue_size=10)
        self.__imu_topic = rospy.Publisher(imu_topic, Imu, queue_size=10)
        self.__looptime_pub = rospy.Publisher(loop_time_topic, Float32, queue_size=10)
        
        rospy.Subscriber(cmd_vel_topic, TwistStamped, callback=self.__cmd_vel_cb, queue_size=10)

        # rospy.Subscriber(cmd_pos_local_topic, )  TODO: MAKE A ROS ACTION SERVER FOR POSITION AND PATHS
        # TODO : MAKE ROS ACTION SERVER FOR POSITION CMD
        # TODO : MAKE ROS ACTION SERVER FOR PATH CMD


        self.__takeoff_service  = rospy.Service(takeoff_service_name, Takeoff, self.__handle_takeoff)
        self.__land_service     = rospy.Service(land_service_name, Land, self.__handle_land)
        self.__wait_service     = rospy.Service(wait_service_name, SetBool, self.__handle_wait) 
        self.__shutdown_service = rospy.Service(shutdown_service_name, SetBool, self.__handle_shutdown)

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

        return self.__vehicle_state

    def __cmd_vel_cb(self, msg):
        """
        Callback for the /cmd/vel ROS topic

        @param msg (geometry_msgs.TwistStamped)
        """

        with self.__flag_lock:
            self.__time_start = time.time()
            self.__finished = False      
            self.__cmd = msg


    def __moveAtVelocity(self):
        """
        Utilize AirSim's Python API to move the drone
        """

        with self.__flag_lock:
            if type(self.__cmd) != TwistStamped:
                self.__cmd = None
                self.__finished = True
                return False

            if self.__cmd.header.frame_id == "local":
                orientation = self.__vehicle_state.kinematics_estimated.orientation

                #print(orientation)
                (pitch, roll, yaw) = airsim.to_eularian_angles(orientation)
                
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

            self.__cmd = None
            self.__finished = True

    def __handle_takeoff(self, req):
        """
        Callback for the rosservice /takeoff. Uses the Python API to takeoff the drone

        @param req (TakeoffRequest) Request on rosservice
        @return (TakeoffResponse) True on successfuly takeoff. Else false.
        """

        with self.__flag_lock:
            self.__finished = False
            self.__cmd = None

            if self.get_state().landed_state == airsim.LandedState.Flying:
                self.__finished = True
                return TakeoffResponse(True)

            time_start = time.time()
            with self.__client_lock:
                self.__client.takeoffAsync(vehicle_name=self.__drone_name)

            if req.waitOnLastTask == False:
                self.__finished = True
                return TakeoffResponse(False)

            while self.get_state().landed_state != airsim.LandedState.Flying and time.time() - time_start < self.__service_timeout:
                time.sleep(0.05)


            self.__finished = True

            if self.__vehicle_state.landed_state == airsim.LandedState.Flying:
                return TakeoffResponse(True)
            else:
                return TakeoffResponse(False)


    def __handle_land(self, req):
        """
        Callback for the rosservice /land. Uses the Python API to land the drone.

        @param req (LandRequest) Request on rosservice \n
        @return (LandResponse) True of succesfully landed. Else false
        """

        with self.__flag_lock:
            self.__finished = False
            self.__cmd = None

            if self.get_state().landed_state == airsim.LandedState.Landed:
                self.__finished = True
                return LandResponse(True)

            time_start = time.time()
            with self.__client_lock:
                self.__client.landAsync(vehicle_name=self.__drone_name)

            if req.waitOnLastTask == False:
                self.__finished = True
                return LandResponse(False)

            while self.get_state().landed_state != airsim.LandedState.Landed and time.time() - time_start < self.__service_timeout:
                time.sleep(0.05)

            self.__finished = True

            if self.__vehicle_state.landed_state == airsim.LandedState.Landed:
                return LandResponse(True)
            else:
                return LandResponse(False)        


    def __handle_wait(self, req):
        """
        Callback for the /wait rosservice. Waits for maximum of self.__pos_cmd_timeout

        @param req (StdBoolRequest) Currently unused\n
        @return (StdBoolResponse) True if command completed. False if timeout
        """

        with self.__flag_lock:
            if self.__finished:
                return SetBoolResponse(True, "")

            while self.__finished == False and time.time() - self.__time_start <= self.__pos_cmd_timeout:
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
        with self.__flag_lock:
            self.__finished = False
            self.__shutdown = True

            with self.__client_lock:
                self.__client.armDisarm(False, vehicle_name=self.__drone_name)
                self.__client.enableApiControl(False, vehicle_name=self.__drone_name)

            self.__finished = True

            return SetBoolResponse(True, "")



    def run(self):
        self.__setup_ros()

        rate = rospy.Rate(100)

        avg_time = 0
        begin_time = time.time()

        i = 0

        while True:
            with self.__flag_lock:
                if self.__shutdown == True or rospy.is_shutdown():
                    break
                
                self.get_state()
                cmd = self.__cmd

            if(type(cmd) == TwistStamped):
                self.__moveAtVelocity()


            avg_time  += time.time() - begin_time
            begin_time = time.time()

            if self.__pub_looptime:
                i += 1

                if i == 100:
                    avg_time /= i
                    msg = Float32(avg_time)
                    self.__looptime_pub.publish(msg)
                    avg_time = 0
                    i = 0

            rate.sleep()

        print(self.__drone_name + " QUITTING")





if __name__ == "__main__":
    if(len(sys.argv) != 2):
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])
    
    client = airsim.MultirotorClient()
    lock = mp.Lock()

    drone = Drone("swarm", drone_name, client, lock)
    drone.start()
    drone.join()