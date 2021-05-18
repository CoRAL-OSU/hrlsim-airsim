#! /usr/bin/python3

from multiprocessing import Lock, Process
import sys, os

import airsim, pymap3d
from airsim.client import MultirotorClient
from airsim.types import MultirotorState, LandedState
import rospy

from pycallgraph import PyCallGraph
from pycallgraph.output import GraphvizOutput

from actionlib import SimpleActionClient
from typing import Dict

from airsim_ros_pkgs.msg import Environment, GPSYaw
from airsim_ros_pkgs.srv import (
    Takeoff,
    TakeoffRequest,
    TakeoffResponse,
    Land,
    LandRequest,
    LandResponse,
)
from airsim_ros_cntrl.msg import (
    Multirotor,
    Sensors,
    State
)

from sensor_msgs.msg import (
    NavSatFix,
    NavSatStatus,
    MagneticField,
    Imu
)

from geometry_msgs.msg import (
    Twist,
    Pose,
    Point,
    Quaternion,
    Accel,
    Vector3
)

from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from std_msgs.msg import Float32, Header


class DroneInfo:
    """
    Class to manage drone info.

    Args:
        DroneName (str): The name of the drone
        process (Drone): Process for a Drone
        pubs (Dict[str, rospy.Publisher], optional): Dict of publishers associated with a drone. Defaults to None.
        subs (Dict[str, rospy.Subscriber], optional): Dict of subscribers associated with a drone. Defaults to None.
        services (Dict[str, rospy.ServiceProxy], optional): Dict of services associated with a drone. Defaults to None.
        actions (Dict[str, SimpleActionClient], optional): Dict of actions associated with a drone. Defaults to None.
    """

    def __init__(
        self,
        DroneName: str,
        process,
        pubs: Dict[str, rospy.Publisher] = None,
        subs: Dict[str, rospy.Subscriber] = None,
        services: Dict[str, rospy.ServiceProxy] = None,
        actions: Dict[str, SimpleActionClient] = None,
    ):
        """
        Constructs info about a drone.
        Used in teams to keep track of all the different ros topics

        Args:
            DroneName (str): The name of the drone
            process (Drone): Process for a Drone
            pubs (Dict[str, rospy.Publisher], optional): Dict of publishers associated with a drone. Defaults to None.
            subs (Dict[str, rospy.Subscriber], optional): Dict of subscribers associated with a drone. Defaults to None.
            services (Dict[str, rospy.ServiceProxy], optional): Dict of services associated with a drone. Defaults to None.
            actions (Dict[str, SimpleActionClient], optional): Dict of actions associated with a drone. Defaults to None.
        """
        self.name = DroneName
        self.process = process
        self.pubs = pubs
        self.subs = subs
        self.services = services
        self.actions = actions
        self.state = Multirotor()


class Drone(Process):
    """
    Super Class for a generalized drone process.
    Each drone has a ros node and generalize topics and services for all types of drones

    All topic names are set up as /swarm_name/drone_name/major_cmd/minor_cmd. Eg. commanding the velocity
    of drone1 in swarm1 would be: /swarm1/drones1/cmd/vel

    All commands are executed through a singluar sim_client that all drones share, therefore, you must aquire the client_lock before using the client.

    Args:
        swarmName (str): The name of the swarm this drone is associated with.
        droneName (str): The name of the drone itself.
        sim_client (airsim.MultirotorClient): The client to use to execture commands.
        client_lock (mp.Lock): The lock for the sim_client.
    """

    def __init__(
        self,
        swarmName: str,
        droneName: str,
        sim_client: MultirotorClient,
        client_lock: Lock,
    ) -> None:
        """
        Constructs a new Drone Process.

        Args:
            swarmName (str): The name of the swarm this drone is associated with.
            droneName (str): The name of the drone itself.
            sim_client (airsim.MultirotorClient): The client to use to execture commands.
            client_lock (mp.Lock): The lock for the sim_client.
        """
        Process.__init__(self)

        self.swarm_name = swarmName
        self.drone_name = droneName
        self.get_sensor_data = False

        self._shutdown = False
        self.__service_timeout = 5.0  # SECONDS

        #self.client_lock = client_lock
        self.flag_lock = Lock()

        self.client_lock = Lock()
        self.client = airsim.MultirotorClient()

        self.freq = 80
        self.prev_loop_time = -999
        self.origin_geo_point = GPSYaw()

        self.cmd = None

        with self.client_lock:
            self.client = sim_client
            self.client.confirmConnection()
            self.client.enableApiControl(True, vehicle_name=self.drone_name)
            self.client.armDisarm(True, vehicle_name=self.drone_name)

        #(self.state, self.sensors) = self.get_state()

        self.state = airsim.MultirotorState()
        self.sensors = dict()
        

    def setup_ros(self) -> None:
        """
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Topics:
            /looptime (Float32): The looptime of the drone

        Services:
            /takeoff (airsim_ros_pkgs.Takeoff): Service to takeoff the drone
            /land (airsim_ros_pkgs.Land): Service to land the drone
            /shutdown (std_srvs.SetBool): Service to shutdown process

        Returns: None
        """
        rospy.init_node(self.drone_name)
        self.topic_prefix = "/" + self.swarm_name + "/" + self.drone_name
        loop_time_topic = self.topic_prefix + "/looptime"

        takeoff_service_name = self.topic_prefix + "/takeoff"
        land_service_name = self.topic_prefix + "/land"
        shutdown_service_name = self.topic_prefix + "/shutdown"

        state_topic = self.topic_prefix + "/multirotor"
        lqr_cmd_topic = "/airsim_node/" + self.drone_name + "/throttle_rates_cmd"

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/airsim_node/origin_geo_point", GPSYaw, callback=self.get_origin)
        rospy.Subscriber("/airsim_node/" + self.drone_name + "/imu/imu0", Imu, callback=self.imu_cb)
        rospy.Subscriber("/airsim_node/" + self.drone_name + "/global_gps", NavSatFix, callback=self.gps_cb)
        rospy.Subscriber("/airsim_node/" + self.drone_name + "/odom_local_ned", Odometry, callback=self.odom_cb)


        self.multirotor_pub = rospy.Publisher(state_topic, Multirotor, queue_size=10)
        self.throttle_rates_cmd_pub = rospy.Publisher(lqr_cmd_topic, Twist, queue_size=1)


        rospy.Service(shutdown_service_name, SetBool, self.__handle_shutdown)
        rospy.Service(takeoff_service_name, Takeoff, self.__handle_takeoff)
        rospy.Service(land_service_name, Land, self.__handle_land)

    def __handle_shutdown(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for the /shutdown rosservice. Uses Python API to disarm drone

        Args:
            req (StdBoolRequest): Currently unused

        Returns (StdBoolResponse): True on success
        """

        print(self.drone_name + " SHUTDOWN REQUEST RECEIVED")
        with self.flag_lock:
            self._shutdown = True

            with self.client_lock:
                self.client.armDisarm(False, vehicle_name=self.drone_name)
                self.client.enableApiControl(False, vehicle_name=self.drone_name)

            print(self.drone_name + " SHUTDOWN REQUEST HANDLED")
            return SetBoolResponse(True, "")

    def __handle_takeoff(self, req: TakeoffRequest) -> TakeoffResponse:
        """
        Callback for the rosservice /takeoff. Uses the Python API to takeoff the drone

        Args:
            req (TakeoffRequest): Request on rosservice

        Returns (TakeoffResponse): True on successfuly takeoff. Else false.
        """
        with self.flag_lock:
            self.cmd = None

            if self.state.landed_state == LandedState.Flying:
                return TakeoffResponse(True)

            time_start = rospy.get_time()
            with self.client_lock:
                self.client.takeoffAsync(vehicle_name=self.drone_name)

            if req.waitOnLastTask == False:
                return TakeoffResponse(False)

            while (
                self.state.landed_state != LandedState.Flying
                and rospy.get_time() - time_start < self.__service_timeout
            ):
                rospy.sleep(0.05)

            if self.state.landed_state == LandedState.Flying:
                return TakeoffResponse(True)
            else:
                return TakeoffResponse(False)

    def __handle_land(self, req: LandRequest) -> LandResponse:
        """
        Callback for the rosservice /land. Uses the Python API to land the drone.

        Args:
            req (LandRequest): Request on rosservice
        
        Returns (LandResponse): True of succesfully landed. Else false
        """

        with self.flag_lock:
            self.cmd = None

            if self.state.landed_state == LandedState.Landed:
                return LandResponse(True)

            time_start = rospy.get_time()
            #with self.client_lock:
            self.client.landAsync(vehicle_name=self.drone_name)

            if req.waitOnLastTask == False:
                return LandResponse(False)

            while (
                self.state.landed_state != LandedState.Landed
                and rospy.get_time() - time_start < self.__service_timeout
            ):
                rospy.sleep(0.05)

            if self.state.landed_state == LandedState.Landed:
                return LandResponse(True)
            else:
                return LandResponse(False)

    def shutdown(self) -> None:
        """
        Handle improper rospy shutdown. Uses Python API to disarm drone

        Returns: None
        """
        with self.flag_lock:
            self._shutdown = True

        print(self.drone_name + " QUITTING")


    def get_origin(self, msg):
        self.origin_geo_point = msg

    def imu_cb(self, msg):
        """
        Get imu msg of drone from airsim_node
        """
        alx = msg.linear_acceleration.x
        aly = msg.linear_acceleration.y
        alz = msg.linear_acceleration.z
        

        qw = msg.orientation.w
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z

        self.state.kinematics_estimated.linear_acceleration = airsim.Vector3r(alx,aly,alz)
        self.state.kinematics_estimated.orientation = airsim.Quaternionr(qx, qy, qz, qw)

    def gps_cb(self, msg):
        """
        Get gps lat/lon of drone from airsim_node
        """
        lat0 = self.origin_geo_point.latitude
        lon0 = self.origin_geo_point.longitude
        alt0 = self.origin_geo_point.altitude

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        (n,e,d) = pymap3d.geodetic2ned(lat, lon, alt, lat0, lon0, alt0)

        self.state.kinematics_estimated.position = airsim.Vector3r(n,e,d)

    def odom_cb(self, msg):
        """
        Get odom of drone from airsim node
        """
        vlx = msg.twist.twist.linear.x
        vly = msg.twist.twist.linear.y
        vlz = msg.twist.twist.linear.z

        vax = msg.twist.twist.angular.x
        vay = msg.twist.twist.angular.y
        vaz = msg.twist.twist.angular.z

        self.state.kinematics_estimated.linear_velocity = airsim.Vector3r(vlx,vly,vlz)
        self.state.kinematics_estimated.angular_velocity = airsim.Vector3r(vax,vay,vaz)


    
    def publish_multirotor_state(self, state, sensors) -> None:
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
        pose = Pose(pos, q)

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


        if self.get_sensor_data:
            # Setup imu msg
            msg.sensors.imu.orientation = Quaternion(*sensors['imu'].orientation.to_numpy_array())
            msg.sensors.imu.angular_velocity = Vector3(*sensors['imu'].angular_velocity.to_numpy_array())
            msg.sensors.imu.linear_acceleration = Vector3(*sensors['imu'].linear_acceleration.to_numpy_array())

            # Setup barometer msg
            msg.sensors.altimeter.altitude = sensors['alt'].altitude
            msg.sensors.altimeter.pressure = sensors['alt'].pressure
            msg.sensors.altimeter.qnh = sensors['alt'].qnh

            # Setup magnetometer msg
            msg.sensors.magnetometer.magnetic_field = Vector3(*sensors['mag'].magnetic_field_body.to_numpy_array())

            # Setup gps msg
            msg.sensors.gps.latitude = sensors['gps'].gnss.geo_point.latitude
            msg.sensors.gps.longitude = sensors['gps'].gnss.geo_point.longitude
            msg.sensors.gps.altitude = sensors['gps'].gnss.geo_point.altitude
            msg.sensors.gps.status.service = NavSatStatus.SERVICE_GLONASS
            msg.sensors.gps.status.status = sensors['gps'].gnss.fix_type
         
  
        # Publish msg
        self.multirotor_pub.publish(msg)
    

    def run(self) -> None:
        """
        Funciton to run when the process starts.

        Returns: None
        """
        self.setup_ros()

        rate = rospy.Rate(self.freq)

        while not rospy.is_shutdown() and self._shutdown == False:
            # (self.state, self.sensors) = self.get_state()
            self.publish_multirotor_state(self.state, self.sensors)

            rate.sleep()

        with self.client_lock:
            self.client.cancelLastTask(vehicle_name=self.drone_name)

        print(self.drone_name + " QUITTING")





if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])

    client = airsim.MultirotorClient(ip="192.168.1.96")
    lock = Lock()

    drone = Drone("swarm", drone_name, client, lock)
    drone.start()
    drone.join()
