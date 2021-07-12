#! /usr/bin/python3

from multiprocessing import Process
import numpy as np

import pymap3d, rospy, actionlib, sys
from airsim.types import MultirotorState, Vector3r, Quaternionr

from actionlib import SimpleActionClient
from typing import Dict

from airsim_ros_pkgs.msg import GPSYaw, VelCmd
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
    State,
    MoveToLocationAction,
    MoveToLocationFeedback,
    MoveToLocationGoal,
    MoveToLocationResult    
)

from sensor_msgs.msg import (
    NavSatFix,
    NavSatStatus,
    Imu
)

from geometry_msgs.msg import (
    Twist, TwistStamped,
    Pose,  PoseStamped,
    Point,
    Quaternion,
    Accel,
    Vector3
)

from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Float32, Header

import lqr


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

    Args:
        swarmName (str): The name of the swarm this drone is associated with.
        droneName (str): The name of the drone itself.
    """

    def __init__(
        self,
        swarmName: str,
        droneName: str
    ) -> None:
        """
        Constructs a new Drone Process.

        Args:
            swarmName (str): The name of the swarm this drone is associated with.
            droneName (str): The name of the drone itself.
        """
        Process.__init__(self)

        self.swarm_name = swarmName
        self.drone_name = droneName
        self.get_sensor_data = False

        self._shutdown = False
        self.__service_timeout = 5.0  # SECONDS

        self.freq = 20
        self.prev_loop_time = -999
        self.origin_geo_point = GPSYaw()

        self.stop_msg = VelCmd()

        self.cmd = None
        self.cmd_timeout = 0.1

        self.controller = lqr.LQR()
        self.prev_accel_cmd = 0

        self.state = MultirotorState()
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
        topic_prefix = self.swarm_name + "/" + self.drone_name
        loop_time_topic = topic_prefix + "/looptime"

        takeoff_service_name = topic_prefix + "/takeoff"
        land_service_name = topic_prefix + "/land"
        shutdown_service_name = topic_prefix + "/shutdown"

        state_topic = topic_prefix + "/multirotor"
        lqr_cmd_topic = "/airsim_node/" + self.drone_name + "/throttle_rates_cmd"

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/airsim_node/origin_geo_point", GPSYaw, callback=self.get_origin)
        rospy.Subscriber("/airsim_node/" + self.drone_name + "/imu/imu0", Imu, callback=self.imu_cb)
        rospy.Subscriber("/airsim_node/" + self.drone_name + "/global_gps", NavSatFix, callback=self.gps_cb)
        rospy.Subscriber("/airsim_node/" + self.drone_name + "/odom_local_ned", Odometry, callback=self.odom_cb)
        rospy.Subscriber(topic_prefix + "/vel_cmd_body_frame", VelCmd, callback=self.vel_cmd_body_frame_cb, queue_size=1)

        self.moveToLocationActionServer = actionlib.SimpleActionServer(
            topic_prefix + "/move_to_location",
            MoveToLocationAction,
            execute_cb=self.move_to_location_cb,
            auto_start=False,
        )
        self.moveToLocationActionServer.start()

        self.vel_cmd_pub = rospy.Publisher("/airsim_node/" + self.drone_name + "/vel_cmd_body_frame", VelCmd, queue_size=2)
        self.multirotor_pub = rospy.Publisher(state_topic, Multirotor, queue_size=2)
        self.throttle_rates_cmd_pub = rospy.Publisher(lqr_cmd_topic, TwistStamped, queue_size=2)
        self.__desired_pose_pub = rospy.Publisher(topic_prefix +"/lqr/desired_pose", PoseStamped, queue_size=2)
        self.__desired_vel_pub = rospy.Publisher(topic_prefix +"/lqr/desired_vel", TwistStamped, queue_size=2)

        rospy.Service(shutdown_service_name, SetBool, self.shutdown_cb)
        rospy.Service(takeoff_service_name, Takeoff, self.takeoff_cb)
        rospy.Service(land_service_name, Land, self.land_cb)

        self.takeoff = rospy.ServiceProxy("/airsim_node/" + self.drone_name + "/takeoff", Takeoff)
        self.land = rospy.ServiceProxy("/airsim_node/" + self.drone_name + "/land", Land)

        self.cmd_timer = rospy.Timer(rospy.Duration(self.cmd_timeout), self.cmd_timer_cb, oneshot=True)

    def cmd_timer_cb(self, event) -> None:
        """
        Callback for the cmd timer. Sets cmd to None
        """
        self.cmd = None
        rospy.loginfo(self.drone_name + ": CMD_TIMER_CB FIRED")


    def shutdown_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for the /shutdown rosservice. Calls process shutdown() method
        """
        self.shutdown()
        return SetBoolResponse(True, "True")

    def takeoff_cb(self, req: TakeoffRequest) -> TakeoffResponse:
        """
        Callback for the rosservice /takeoff. Passes to AirSim ROS Wrapper
        """
        self.cmd_timer.shutdown()
        self.cmd = Takeoff()
        return TakeoffResponse(True)

    def land_cb(self, req: LandRequest) -> LandResponse:
        """
        Callback for the rosservice /land. Passes to AirSim ROS Wrapper
        """
        self.cmd_timer.shutdown()
        self.cmd = Land()
        return LandResponse(True)

    def shutdown(self) -> None:
        """
        Handle improper rospy shutdown.
        """
        self.cmd_timer.shutdown()
        self.cmd = None
        self.vel_cmd_pub.publish(self.stop_msg)
        self._shutdown = True

    def get_origin(self, msg):
        self.origin_geo_point = msg

    def imu_cb(self, msg):
        """
        Get imu msg of drone from airsim_node
        """
        alx = msg.linear_acceleration.x
        aly = msg.linear_acceleration.y
        alz = msg.linear_acceleration.z

        vax = msg.angular_velocity.x
        vay = msg.angular_velocity.y
        vaz = msg.angular_velocity.z
        

        qw = msg.orientation.w
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z

        self.state.kinematics_estimated.linear_acceleration = Vector3r(alx,aly,alz)
        self.state.kinematics_estimated.orientation = Quaternionr(qx, qy, qz, qw)
        self.state.kinematics_estimated.angular_velocity = Vector3r(vax, vay, vaz)

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

        self.state.kinematics_estimated.position = Vector3r(n,e,d)

    def odom_cb(self, msg):
        """
        Get odom of drone from airsim node
        """
        vlx = msg.twist.twist.linear.x
        vly = msg.twist.twist.linear.y
        vlz = msg.twist.twist.linear.z

        self.state.kinematics_estimated.linear_velocity = Vector3r(vlx,vly,vlz)

    def vel_cmd_body_frame_cb(self, msg):
        """
        Handle velocity command in the body frame. Currently just passes to AirSim ROS Wrapper. ROS timer 
        sets self.cmd to None after elapsed period
        """
        self.cmd = msg
        self.cmd_timer.shutdown()
        self.cmd_timer = rospy.Timer(rospy.Duration(self.cmd_timeout), self.cmd_timer_cb, oneshot=True)

    def move_to_location_cb(self, goal: MoveToLocationGoal) -> MoveToLocationResult:
        """
        Handle position command in the world frame
        """
        self.cmd_timer.shutdown()

        pt = np.array(goal.target)
        p = self.state.kinematics_estimated.position.to_numpy_array()

        if goal.position_frame == MoveToLocationGoal.GLOBAL_FRAME:
            pass
        elif goal.position_frame == MoveToLocationGoal.LOCAL_FRAME:
            pt += p

        waypoints = np.array([p,pt]).T
        
        v = self.state.kinematics_estimated.linear_velocity.to_numpy_array()
        a = self.state.kinematics_estimated.linear_acceleration.to_numpy_array()
        j = np.zeros(3)
        ic = np.array([v, a, j])

        fv = np.array(goal.fvel)
        fa = np.array(goal.facc)
        fj = np.array(goal.fjrk)
        fc = np.array([fv,fa,fj])

        self.controller.set_goals(waypoints, ic, fc, goal.speed)

        d = np.linalg.norm(pt-p)
        t0 = rospy.get_time()

        self.t0 = t0

        feedback = MoveToLocationFeedback()

        self.cmd = goal

        r = rospy.Rate(self.freq)
        while d > goal.tolerance and rospy.get_time() - t0 < goal.timeout and not self.moveToLocationActionServer.is_preempt_requested() and isinstance(self.cmd, MoveToLocationGoal):
            p = self.state.kinematics_estimated.position.to_numpy_array()
            d = np.linalg.norm(pt-p)

            for i in range(0,len(p)):
                feedback.location[i] = p[i]

            feedback.error = d
            feedback.time_left = rospy.get_time() - t0

            r.sleep()

        self.cmd = None

        if d < goal.tolerance:
            self.moveToLocationActionServer.set_succeeded(feedback)
            rospy.loginfo(self.drone_name + ": MOVE_TO_LOCATION SUCCESSFUL")
        else:
            self.moveToLocationActionServer.set_aborted(feedback)
            rospy.logwarn(self.drone_name + ": MOVE_TO_LOCATION FAILED")



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
    
    ##                                   ##
    ###        LQR IMPLEMENATION        ###
    #######################################

    def moveByLQR(self, t0: float, state: MultirotorState) -> None:
        """
        Moves the agent via LQR.

        Args:
            t (float): Time elapsed since beginning
            state (MultirotorState): Current Multirotor State
        """
        x0, u = self.controller.computeControl(t0, state, self.prev_accel_cmd, self.drone_name)

        for i in range(0, 3):
            u[i, 0] = max(-3, u[i, 0])
            u[i, 0] = min(3, u[i, 0])

        u[3, 0] = min(u[3, 0], 1)

        roll_rate  = u[0, 0]
        pitch_rate = u[1, 0]
        yaw_rate   = u[2, 0]
        throttle   = u[3, 0]

        accel = self.controller.thrust2world(state, throttle)

        self.prev_acceleration_cmd = throttle

        throttle_rates_cmd = TwistStamped()
        throttle_rates_cmd.header.stamp = rospy.Time.now()
        throttle_rates_cmd.twist.linear.z = throttle
        throttle_rates_cmd.twist.angular.x = roll_rate
        throttle_rates_cmd.twist.angular.y = pitch_rate
        throttle_rates_cmd.twist.angular.z = yaw_rate

        self.throttle_rates_cmd_pub.publish(throttle_rates_cmd)

        
        (pDes, qDes, vDes) = lqr.LQR.get_state(x0)
        desired_pose_msg = PoseStamped()
        desired_pose_msg.header.stamp = rospy.Time.now()
        desired_pose_msg.pose.position = Point(*pDes)
        desired_pose_msg.pose.orientation = Quaternion(w=qDes[0], x=qDes[1], y=qDes[2], z=qDes[3])
        self.__desired_pose_pub.publish(desired_pose_msg)

        desired_vel_msg = TwistStamped()
        desired_vel_msg.header.stamp = rospy.Time.now()
        desired_vel_msg.twist.linear = Vector3(*vDes)
        desired_vel_msg.twist.angular = Vector3(*u[0:3,0])
        self.__desired_vel_pub.publish(desired_vel_msg)



    ##                                   ##
    ###        Main loop                ###
    #######################################

    def run(self) -> None:
        """
        Function to run when the process starts.

        Returns: None
        """
        self.setup_ros()

        rate = rospy.Rate(self.freq)

        while not rospy.is_shutdown() and self._shutdown == False:
            self.publish_multirotor_state(self.state, self.sensors)

            if self.cmd == None:
                self.vel_cmd_pub.publish(self.stop_msg)

            elif isinstance(self.cmd, VelCmd):
                self.vel_cmd_pub.publish(self.cmd)

            elif isinstance(self.cmd, Takeoff):
                self.takeoff()
                self.cmd == None

            elif isinstance(self.cmd, Land):
                self.land()
                self.cmd == None

            elif isinstance(self.cmd, MoveToLocationGoal):
                self.moveByLQR(self.t0, self.state)

            rate.sleep()

        print(self.drone_name + " QUITTING")




if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])

    drone = Drone("Team0", drone_name)
    drone.start()
    drone.join()
