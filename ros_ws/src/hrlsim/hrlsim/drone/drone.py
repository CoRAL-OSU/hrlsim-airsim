#! /usr/bin/python3

import threading
import pymap3d

import time

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from airsim_interfaces.msg import GPSYaw, VelCmd
from airsim_interfaces.srv import Takeoff, Land

from hrlsim_interfaces.msg import Multirotor, State
from hrlsim_interfaces.action import MoveToLocation

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu

from geometry_msgs.msg import (
    Twist, TwistStamped,
    Pose,  PoseStamped,
    Point,
    Quaternion,
    Accel,
    Vector3
)

from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_msgs.msg import Header

import hrlsim
import hrlsim.controller
import hrlsim.traj
import hrlsim.utility

class DroneInfo:
    """
    Class to manage drone info.
    """

    def __init__(self, DroneName, process, pubs = None, subs = None, services = None, actions = None):
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








class Drone(Node):
    """
    Super Class for a generalized drone process.
    Each drone has a ros node and generalize topics and services for all types of drones

    All topic names are set up as /swarm_name/drone_name/major_cmd/minor_cmd. Eg. commanding the velocity
    of drone1 in swarm1 would be: /swarm1/drones1/cmd/vel
    """

    def __init__(self, swarmName: str, droneName: str, controllerType: hrlsim.controller.Controller, trajType: hrlsim.traj.Trajectory, executor = None, maxThrust=16.7176, mass=1) -> None:
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
        super().__init__(swarmName+"_"+droneName)

        self.get_logger().info("Spinning up multithreaded exector")
        
        self.swarm_name = swarmName
        self.drone_name = droneName
        self.cmd = None
        self.cmd_timeout = 0.1        
        self.get_sensor_data = False

        self.mass = mass # kg
        self.maxThrust = maxThrust  # N

        self._shutdown = False

        self.freq = 100
        self.prev_loop_time = self.get_clock().now()
        self.actual_loop_time = 0
        self.origin_geo_point = GPSYaw()

        self.stop_msg = VelCmd()

        self.controller = controllerType(trajType, self.maxThrust, self.mass)

        self.state = hrlsim.airsim.MultirotorState()
        self.sensors = dict()

        self.run()

        

    def setup_ros(self) -> None:
        """
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Returns: None
        """
        airsim_prefix = "/airsim_node/" + self.drone_name

        cbGroup = ReentrantCallbackGroup()

        # Subscribed topics
        self.create_subscription(GPSYaw, "/airsim_node/origin_geo_point", self.get_origin_cb, 10, callback_group=cbGroup)
        self.create_subscription(Imu, airsim_prefix + "/imu/imu0", self.imu_cb, 10, callback_group=cbGroup)
        self.create_subscription(NavSatFix, airsim_prefix + "/global/gps", self.gps_cb, 10, callback_group=cbGroup)
        self.create_subscription(Odometry, airsim_prefix + "/odom/local_ned", self.odom_cb, 10, callback_group=cbGroup)
        self.create_subscription(VelCmd, "~/vel_cmd_body_frame", self.vel_cmd_body_frame_cb, 10, callback_group=cbGroup)

        # Published action servers
        self.moveToLocationActionServer = ActionServer(
            self,
            MoveToLocation,
            "~/move_to_location",
            self.move_to_location_cb,
            callback_group=cbGroup
        )


        # Published topics
        self.vel_cmd_pub            = self.create_publisher(VelCmd, "~/vel_cmd_body_frame", 2)
        self.throttle_rates_cmd_pub = self.create_publisher(TwistStamped, "~/throttle_rates_cmd", 2)
        self.multirotor_pub         = self.create_publisher(Multirotor, "~/multirotor", 2)
        self.desired_pose_pub     = self.create_publisher(PoseStamped, "~/lqr/desired_pose", 2)
        self.desired_vel_pub      = self.create_publisher(TwistStamped, "~/lqr/desired_vel", 2)
        
        # Published services
        self.create_service(SetBool, "~/shutdown", self.shutdown_cb, callback_group=cbGroup)
        self.create_service(Takeoff, "~/takeoff", self.takeoff_cb, callback_group=cbGroup)
        self.create_service(Land, "~/land", self.land_cb, callback_group=cbGroup)

        # Subscribed services
        self.takeoff    = self.create_client(Takeoff, airsim_prefix + "/takeoff")
        self.land       = self.create_client(Land, airsim_prefix + "/land")

        self.cmd_timer = self.create_timer(self.cmd_timeout, self.cmd_timer_cb, callback_group=cbGroup)
        self.cmd_timer.cancel()

        self.loop_timer = self.create_timer(1/self.freq, self.loop, callback_group=cbGroup)

        self.executor = MultiThreadedExecutor(4)#executor
        if self.executor != None:
            self.exector.add_node(self)

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
        self.get_logger().info(self.drone_name + ": CMD_TIMER_CB FIRED")


    def shutdown_cb(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        """
        Callback for the /shutdown rosservice. Calls process shutdown() method
        """
        self.shutdown()
        resp.success = True
        return resp

    def takeoff_cb(self, req: Takeoff.Request, resp: Takeoff.Response) -> Takeoff.Response:
        """
        Callback for the rosservice /takeoff. Passes to AirSim ROS Wrapper
        """
        self.state.landed_state = hrlsim.airsim.LandedState.Flying
        self.resetCmdTimer()
        self.cmd = req
        print(self.cmd)

        resp.success = True
        return resp

    def land_cb(self, req: Land.Request, resp: Land.Response) -> Land.Response:
        """
        Callback for the rosservice /land. Passes to AirSim ROS Wrapper
        """
        self.state.landed_state = hrlsim.airsim.LandedState.Landed
        self.resetCmdTimer()
        self.cmd = req

        resp.success = True
        return resp

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
        self.cmd_timer.reset()

    def attitude_body_frame_cb(self, msg):
        # handle body angle cmd
        print(self.msg)


    #
    ###################################
    #
    #   ACTION SERVER CALLBACK FUNCTIONS
    #

    def move_to_location_cb(self, goal_handle):
        """
        Handle position command in the world frame
        """
        self.get_logger().info("Entering MoveToLocation with goal")
        self.resetCmdTimer()

        self.state.landed_state = hrlsim.airsim.LandedState.Flying

        pt = np.array(goal_handle.request.target)
        p = self.state.kinematics_estimated.position.to_numpy_array()

        if goal_handle.request.position_frame == MoveToLocation.Goal.GLOBAL_FRAME:
            pass
        elif goal_handle.request.position_frame == MoveToLocation.Goal.LOCAL_FRAME:
            pt += p

        # Setup waypoints for trajectory (current position -> goal_handle.request position)
        waypoints = np.array([p,pt]).T
        
        # Setup initial conditions (from current state)
        v = self.state.kinematics_estimated.linear_velocity.to_numpy_array()
        a = self.state.kinematics_estimated.linear_acceleration.to_numpy_array()
        j = np.zeros(3)
        ic = np.array([v, a, j])

        # Setup final conditions (from target state)
        fv = np.array(goal_handle.request.fvel)
        fa = np.array(goal_handle.request.facc)
        fj = np.array(goal_handle.request.fjrk)
        fc = np.array([fv,fa,fj])

        # Send to controller
        self.controller.setGoals(waypoints, ic, fc, goal_handle.request.speed)

        d = np.linalg.norm(pt-p)
        self.t0 = self.get_clock().now().nanoseconds/1e9
        tnow = self.t0

        feedback = MoveToLocation.Feedback()

        self.cmd = goal_handle.request

        r = self.create_rate(self.freq)
        while isinstance(self.cmd, MoveToLocation.Goal) and d > self.cmd.tolerance and tnow-self.t0 < self.cmd.timeout:
            p = self.state.kinematics_estimated.position.to_numpy_array()
            d = float(np.linalg.norm(pt-p))

            for i in range(0,len(p)):
                feedback.location[i] = p[i]

            feedback.error = d
            tnow = self.get_clock().now().nanoseconds/1e9
            feedback.time_left = tnow-self.t0


            self.get_logger().info("Waiting at rate")

            r.sleep()
            #time.sleep(1/self.freq)

        self.cmd = None


        if d < goal_handle.request.tolerance:
            goal_handle.succeed()
            self.get_logger().info(self.drone_name + ": MOVE_TO_LOCATION SUCCESSFUL")
        else:
            goal_handle.abort()
            self.get_logger().info(self.drone_name + ": MOVE_TO_LOCATION FAILED")

        result = MoveToLocation.Result()
        result.error = feedback.error
        result.location = feedback.location
        result.time_left = feedback.time_left

        return result

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
        msg.header.stamp = self.get_clock().now().to_msg()

        self.actual_loop_time = (self.get_clock().now() - self.prev_loop_time).nanoseconds/1.0e9

        msg.looptime.data = self.actual_loop_time
        self.prev_loop_time = self.get_clock().now()


        # Setup pose msg
        pos = Point(x=state.position.x_val, y=state.position.y_val, z=state.position.z_val)
        q = Quaternion(x=state.orientation.x_val, y=state.orientation.y_val, z=state.orientation.z_val, w=state.orientation.w_val)      
        pose = Pose(position=pos, orientation=q)

        # Setup twist msg
        lin_vel = Vector3(x=state.linear_velocity.x_val, y=state.linear_velocity.y_val, z=state.linear_velocity.z_val)
        ang_vel = Vector3(x=state.angular_velocity.x_val, y=state.angular_velocity.y_val, z=state.angular_velocity.z_val)
        twist = Twist(linear=lin_vel, angular=ang_vel)

        # Setup acc msg
        lin_acc = Vector3(x=state.linear_acceleration.x_val, y=state.linear_acceleration.y_val, z=state.linear_acceleration.z_val)
        ang_acc = Vector3(x=state.angular_acceleration.x_val, y=state.angular_acceleration.y_val, z=state.angular_acceleration.z_val)
        acc = Accel(linear=lin_acc, angular=ang_acc)

        # Make state msg
        msg.state = State(pose=pose, vel=twist, acc=acc)

        if self.get_sensor_data:
            # Setup imu msg
            msg.sensors.imu.orientation = Quaternion(x=sensors['imu'].orientation.x_val, y=sensors['imu'].orientation.y_val, z=sensors['imu'].orientation.z_val)
            msg.sensors.imu.angular_velocity = Vector3(x=sensors['imu'].angular_velocity.x_val, y=sensors['imu'].angular_velocity.y_val, z=sensors['imu'].angular_velocity.z_val)
            msg.sensors.imu.linear_acceleration = Vector3(x=sensors['imu'].linear_acceleration.x_val, y=sensors['imu'].linear_acceleraiton.y_val, z=sensors['imu'].linear_accelerations.z_val)

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

        x0, u = self.controller.computeControl(self.get_clock().now().nanoseconds/1e9, t0, self.actual_loop_time, state, self.drone_name)

        # Bound control output to +/- 2 [rad/s]
        u = np.maximum(u, -2*np.ones(u.shape))
        u = np.minimum(u,  2*np.ones(u.shape))
        u[3,0] = min(u[3,0], 1)

        # Publish command to low-level controls
        (wp,wq,wr,th) = hrlsim.utility.cmd2comp(u)
        throttle_rates_cmd = TwistStamped()
        throttle_rates_cmd.header.stamp = self.get_clock().now().to_msg()
        throttle_rates_cmd.twist.linear.z  = th # Throttle
        throttle_rates_cmd.twist.angular = Vector3(x=wp, y=wq, z=wr)

        self.throttle_rates_cmd_pub.publish(throttle_rates_cmd)
        
        # Publish reference trajectory for data collection
        (pDes, qDes, vDes) = hrlsim.utility.state2comp(x0)
        desired_pose_msg = PoseStamped()
        desired_pose_msg.header.stamp = self.get_clock().now().to_msg()
        desired_pose_msg.pose.position = Point(x=pDes[0], y=pDes[1], z=pDes[2])
        desired_pose_msg.pose.orientation = Quaternion(w=qDes[0], x=qDes[1], y=qDes[2], z=qDes[3])
        self.desired_pose_pub.publish(desired_pose_msg)

        desired_vel_msg = TwistStamped()
        desired_vel_msg.header.stamp = self.get_clock().now().to_msg()
        desired_vel_msg.twist.linear = Vector3(x=vDes[0], y=vDes[1], z=vDes[2])
        desired_vel_msg.twist.angular = Vector3(x=wp, y=wq, z=wr)
        self.desired_vel_pub.publish(desired_vel_msg)



    ##                                   ##
    ###        Main loop                ###
    #######################################
    def loop(self) -> None:           
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

                if self.first:
                    self.controller.setGoals(waypoints, ic, fc, 1.0)
                    self.first = False
                
                t0 = self.get_clock().now().nanoseconds/1e9
                self.move(t0, self.state)

            else:
                pass
                #self.vel_cmd_pub.publish(self.stop_msg)

        elif isinstance(self.cmd, VelCmd):
            self.first = True
            self.vel_cmd_pub.publish(self.cmd)

        elif isinstance(self.cmd, Takeoff.Request):
            self.first = True
            print(self.cmd)
            self.takeoff.call(self.cmd)
            self.cmd == None

        elif isinstance(self.cmd, Land.Request):
            self.first = True
            self.land.call(self.cmd)
            self.cmd == None

        elif isinstance(self.cmd, MoveToLocation.Goal):
            self.first = True
            self.move(self.t0, self.state)




    def run(self) -> None:
        """
        Function to run when the process starts.

        Returns: None
        """
        self.first = True

        self.get_logger().info("Setting up ROS")
        self.setup_ros()

        #self.get_logger().info("Setting up spinner thread")
        #spinner = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        #spinner.start()

        rate = self.create_rate(self.freq)


        self.get_logger().info("Entering main loop")

        if self.executor != None:
            self.executor.spin()
        else:
            rclpy.spin(self)

            
        #self.get_logger().info(self.drone_name + " QUITTING")
        #self.moveToLocationActionServer.destroy()
        #self.destroy_node()

if __name__ == "__main__":

    from concurrent.futures import ProcessPoolExecutor

    rclpy.init()

    names = ['0']

    print("launching drones")

    try:
        #executor = MultiThreadedExecutor(num_threads=4)
        with ProcessPoolExecutor(len(names)) as pool:
            futures = {pool.submit(Drone, "swarm", "Drone"+str(name), hrlsim.controller.LQR, hrlsim.traj.MinimumSnap): name for name in names}

    finally:
        print("parent exiting")
        #executor.shutdown()
        rclpy.shutdown()
