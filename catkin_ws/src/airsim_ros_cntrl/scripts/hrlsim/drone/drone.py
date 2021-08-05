#! /usr/bin/python3

from multiprocessing import Process
import numpy as np

import pymap3d, rospy, actionlib

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

import hrlsim

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

    def __init__(self,
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
        self.state = hrlsim.airsim.Multirotor()


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

    def __init__(self, swarmName: str, droneName: str, controllerType: hrlsim.controller.Controller, trajType: hrlsim.traj.Trajectory, maxThrust=16.7176, mass=1) -> None:
        """
        Constructs a new Drone Process.

        Args:
            swarmName (str): The name of the swarm this drone is associated with.
            droneName (str): The name of the drone itself.
            controller (Controller): The position controller
            trajGenerator (Trajectory): The trajectory generation
            maxThrust (float): Maximal thrust deliverable by rotors (N)
            mass (float): Mass of drone (kg)
        """
        Process.__init__(self)


        self.mass = mass # kg
        self.maxThrust = maxThrust  # N

        self.swarm_name = swarmName
        self.drone_name = droneName
        self.get_sensor_data = False

        self._shutdown = False

        self.freq = 100
        self.prev_loop_time = -999
        self.actual_loop_time = 0
        self.origin_geo_point = GPSYaw()

        self.stop_msg = VelCmd()

        self.cmd = None
        self.cmd_timeout = 0.1

        self.trajType = trajType
        self.controller = controllerType(trajType, self.maxThrust, self.mass)

        self.state = hrlsim.airsim.MultirotorState()
        self.sensors = dict()

        

    def setup_ros(self) -> None:
        """
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Returns: None
        """
        rospy.init_node(self.drone_name)
        lowlevel_topic_prefix = "/airsim_node/" + self.drone_name
        topic_prefix = self.swarm_name + "/" + self.drone_name

        rospy.on_shutdown(self.shutdown)

        # Subscribed topics
        rospy.Subscriber("/airsim_node/origin_geo_point", GPSYaw, callback=self.get_origin_cb)
        rospy.Subscriber(lowlevel_topic_prefix + "/imu/imu0", Imu, callback=self.imu_cb)
        rospy.Subscriber(lowlevel_topic_prefix + "/global_gps", NavSatFix, callback=self.gps_cb)
        rospy.Subscriber(lowlevel_topic_prefix + "/odom_local_ned", Odometry, callback=self.odom_cb)
        rospy.Subscriber(topic_prefix + "/vel_cmd_body_frame", VelCmd, callback=self.vel_cmd_body_frame_cb, queue_size=1)

        # Published action servers
        self.moveToLocationActionServer = actionlib.SimpleActionServer(
            topic_prefix + "/move_to_location",
            MoveToLocationAction,
            execute_cb=self.move_to_location_cb,
            auto_start=False,
        )
        self.moveToLocationActionServer.start()

        # Published topics
        self.vel_cmd_pub            = rospy.Publisher(lowlevel_topic_prefix + "/vel_cmd_body_frame", VelCmd, queue_size=2)
        self.throttle_rates_cmd_pub = rospy.Publisher(lowlevel_topic_prefix + "/throttle_rates_cmd", TwistStamped, queue_size=2)
        self.multirotor_pub         = rospy.Publisher(topic_prefix+"/multirotor", Multirotor, queue_size=2)
        self.__desired_pose_pub     = rospy.Publisher(topic_prefix + "/lqr/desired_pose", PoseStamped, queue_size=2)
        self.__desired_vel_pub      = rospy.Publisher(topic_prefix + "/lqr/desired_vel", TwistStamped, queue_size=2)

        # Published services
        rospy.Service(topic_prefix+"/shutdown", SetBool, self.shutdown_cb)
        rospy.Service(topic_prefix+"/takeoff",  Takeoff, self.takeoff_cb)
        rospy.Service(topic_prefix+"/land",     Land, self.land_cb)

        # Subscribed services
        self.takeoff = rospy.ServiceProxy("/airsim_node/" + self.drone_name + "/takeoff", Takeoff)
        self.land    = rospy.ServiceProxy("/airsim_node/" + self.drone_name + "/land", Land)

        self.cmd_timer = rospy.Timer(rospy.Duration(self.cmd_timeout), self.cmd_timer_cb, oneshot=True)

  
    #
    ###################################
    #
    #   TOPIC CALLBACK FUNCTIONS
    #

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
        self.state.landed_state = hrlsim.airsim.LandedState.Flying
        self.resetCmdTimer()
        self.cmd = Takeoff()
        return TakeoffResponse(True)

    def land_cb(self, req: LandRequest) -> LandResponse:
        """
        Callback for the rosservice /land. Passes to AirSim ROS Wrapper
        """
        self.state.landed_state = hrlsim.airsim.LandedState.Landed
        self.resetCmdTimer()
        self.cmd = Land()
        return LandResponse(True)

    def get_origin_cb(self, msg):
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

        self.state.kinematics_estimated.linear_acceleration = hrlsim.airsim.Vector3r(alx,aly,alz)
        self.state.kinematics_estimated.orientation = hrlsim.airsim.Quaternionr(qx, qy, qz, qw)
        self.state.kinematics_estimated.angular_velocity = hrlsim.airsim.Vector3r(vax, vay, vaz)

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

        self.state.kinematics_estimated.position = hrlsim.airsim.Vector3r(n,e,d)

    def odom_cb(self, msg):
        """
        Get odom of drone from airsim node
        """
        vlx = msg.twist.twist.linear.x
        vly = msg.twist.twist.linear.y
        vlz = msg.twist.twist.linear.z

        self.state.kinematics_estimated.linear_velocity = hrlsim.airsim.Vector3r(vlx,vly,vlz)

    def vel_cmd_body_frame_cb(self, msg):
        """
        Handle velocity command in the body frame. Currently just passes to AirSim ROS Wrapper. ROS timer 
        sets self.cmd to None after elapsed period
        """
        self.state.landed_state = hrlsim.airsim.LandedState.Flying
        self.cmd = msg
        self.resetCmdTimer()
        self.cmd_timer = rospy.Timer(rospy.Duration(self.cmd_timeout), self.cmd_timer_cb, oneshot=True)


    #
    ###################################
    #
    #   ACTION SERVER CALLBACK FUNCTIONS
    #

    def move_to_location_cb(self, goal: MoveToLocationGoal) -> MoveToLocationResult:
        """
        Handle position command in the world frame
        """
        self.resetCmdTimer()

        self.state.landed_state = hrlsim.airsim.LandedState.Flying

        pt = np.array(goal.target)
        p = self.state.kinematics_estimated.position.to_numpy_array()

        if goal.position_frame == MoveToLocationGoal.GLOBAL_FRAME:
            pass
        elif goal.position_frame == MoveToLocationGoal.LOCAL_FRAME:
            pt += p

        # Setup waypoints for trajectory (current position -> goal position)
        waypoints = np.array([p,pt]).T
        
        # Setup initial conditions (from current state)
        v = self.state.kinematics_estimated.linear_velocity.to_numpy_array()
        a = self.state.kinematics_estimated.linear_acceleration.to_numpy_array()
        j = np.zeros(3)
        ic = np.array([v, a, j])

        # Setup final conditions (from target state)
        fv = np.array(goal.fvel)
        fa = np.array(goal.facc)
        fj = np.array(goal.fjrk)
        fc = np.array([fv,fa,fj])

        # Send to controller
        self.controller.setGoals(waypoints, ic, fc, goal.speed)

        d = np.linalg.norm(pt-p)
        self.t0 = rospy.get_time()

        feedback = MoveToLocationFeedback()

        self.cmd = goal

        r = rospy.Rate(self.freq)
        while d > goal.tolerance and rospy.get_time() - self.t0 < goal.timeout and not self.moveToLocationActionServer.is_preempt_requested() and isinstance(self.cmd, MoveToLocationGoal):
            p = self.state.kinematics_estimated.position.to_numpy_array()
            d = np.linalg.norm(pt-p)

            for i in range(0,len(p)):
                feedback.location[i] = p[i]

            feedback.error = d
            feedback.time_left = rospy.get_time() - self.t0

            r.sleep()

        self.cmd = None

        if d < goal.tolerance:
            self.moveToLocationActionServer.set_succeeded(feedback)
            rospy.loginfo(self.drone_name + ": MOVE_TO_LOCATION SUCCESSFUL")
        else:
            self.moveToLocationActionServer.set_aborted(feedback)
            rospy.logwarn(self.drone_name + ": MOVE_TO_LOCATION FAILED")


    #
    ###################################
    #
    #   UTILITY FUNCTIONS
    #

    def resetCmdTimer(self):
        try:
            self.cmd_timer.shutdown()
        except:
            pass
        return

    def shutdown(self) -> None:
        """
        Handle rospy shutdown.
        """
        self.state.landed_state = hrlsim.airsim.LandedState.Landed
        self.resetCmdTimer()
        self.cmd = None
        self.vel_cmd_pub.publish(self.stop_msg)
        self._shutdown = True

    def publish_multirotor_state(self, state, sensors) -> None:
        """
        Function to publish sensor/state information from the simulator
        Returns: None
        """

        state = state.kinematics_estimated
        msg = Multirotor()
        
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        self.actual_loop_time = rospy.get_time() - self.prev_loop_time
        msg.looptime = Float32(self.actual_loop_time)
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
    ###      CONTROL IMPLEMENATION      ###
    #######################################

    def move(self, t0: float, state: hrlsim.airsim.MultirotorState) -> None:
        """
        Moves the agent via the motion controller.

        Args:
            t (float): Time elapsed since beginning
            state (MultirotorState): Current Multirotor State
        """

        x0, u = self.controller.computeControl(t0, self.actual_loop_time, state, self.drone_name)

        # Bound control output to +/- 2 [rad/s]
        u = np.maximum(u, -2*np.ones(u.shape))
        u = np.minimum(u,  2*np.ones(u.shape))
        u[3,0] = min(u[3,0], 1)

        # Publish command to low-level controls
        (wp,wq,wr,th) = hrlsim.utility.cmd2comp(u)
        throttle_rates_cmd = TwistStamped()
        throttle_rates_cmd.header.stamp = rospy.Time.now()
        throttle_rates_cmd.twist.linear.z  = th # Throttle
        throttle_rates_cmd.twist.angular = Vector3(*[wp,wq,wr])

        self.throttle_rates_cmd_pub.publish(throttle_rates_cmd)
        
        # Publish reference trajectory for data collection
        (pDes, qDes, vDes) = hrlsim.utility.state2comp(x0)
        desired_pose_msg = PoseStamped()
        desired_pose_msg.header.stamp = rospy.Time.now()
        desired_pose_msg.pose.position = Point(*pDes)
        desired_pose_msg.pose.orientation = Quaternion(w=qDes[0], x=qDes[1], y=qDes[2], z=qDes[3])
        self.__desired_pose_pub.publish(desired_pose_msg)

        desired_vel_msg = TwistStamped()
        desired_vel_msg.header.stamp = rospy.Time.now()
        desired_vel_msg.twist.linear = Vector3(*vDes)
        desired_vel_msg.twist.angular = Vector3(*[wp,wq,wr])
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

        first = True

        while not rospy.is_shutdown() and self._shutdown == False:
            self.publish_multirotor_state(self.state, self.sensors)

            if self.cmd == None:
                if self.state.landed_state != hrlsim.airsim.LandedState.Landed:
                    p = self.state.kinematics_estimated.position.to_numpy_array()
                    waypoints = np.array([p]).T
            
                    v = self.state.kinematics_estimated.linear_velocity.to_numpy_array()
                    a = self.state.kinematics_estimated.linear_acceleration.to_numpy_array()
                    j = np.zeros(3)
                    ic = np.array([v, a, j])

                    fc = np.zeros((3,3))

                    if first:
                        self.controller.setGoals(waypoints, ic, fc, 1.0)
                        first = False
                    
                    self.t0 = rospy.get_time()
                    self.move(self.t0, self.state)

                else:
                    pass
                    #self.vel_cmd_pub.publish(self.stop_msg)

            elif isinstance(self.cmd, VelCmd):
                first = True
                self.vel_cmd_pub.publish(self.cmd)

            elif isinstance(self.cmd, Takeoff):
                first = True
                self.takeoff()
                self.cmd == None

            elif isinstance(self.cmd, Land):
                first = True
                self.land()
                self.cmd == None

            elif isinstance(self.cmd, MoveToLocationGoal):
                first = True
                self.move(self.t0, self.state)

            rate.sleep()

        print(self.drone_name + " QUITTING")