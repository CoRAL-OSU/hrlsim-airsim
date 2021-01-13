#! /usr/bin/python2

import time
import multiprocessing as mp
import numpy as np
import math
import sys, os
import matplotlib.pyplot as plt

import airsim
import rospy, actionlib

import lowlevel

from geometry_msgs.msg import TwistStamped, PoseStamped
from airsim_ros_pkgs.srv import Takeoff, TakeoffResponse, Land, LandResponse

from std_srvs.srv import SetBool, SetBoolResponse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from std_msgs.msg import Float32

from airsim_ros_cntrl.msg import TrackObjectAction, TrackObjectFeedback, TrackObjectResult, TrackObjectGoal
from airsim_ros_cntrl.msg import MoveToLocationAction, MoveToLocationFeedback, MoveToLocationResult, MoveToLocationGoal


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

# Calculate vector1 - vector2
def calc_distance(vector1, vector2):
    output = airsim.Vector3r()

    output.x_val = vector1.x_val - vector2.x_val
    output.y_val = vector1.y_val - vector2.y_val
    output.z_val = vector1.z_val - vector2.z_val
    return output




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

        self.__mass = 1.0 # kg 
        self.__max_thrust = 4.1794 * 4 # N

        self.__client_lock = client_lock
        self.__flag_lock = mp.Lock()

        self.__command_type = None
        self.__cmd = None

        self.__controller = lowlevel.LQR() 

        with self.__client_lock:
            self.__client = sim_client
            self.__client.confirmConnection()
            self.__client.enableApiControl(True, vehicle_name=self.__drone_name)
            self.__client.armDisarm(True, vehicle_name=self.__drone_name)

            #x_y_gains = airsim.PIDGains(0.2, 0.0, 0.05)
            #z_gains = airsim.PIDGains(4.0, 2.0, 0.0)
            #vel_pid_gains = airsim.VelocityControllerGains(x_gains=x_y_gains, y_gains=x_y_gains, z_gains=z_gains)

            #self.__client.setVelocityControllerGains(velocity_gains=vel_pid_gains, vehicle_name=self.__drone_name)

        self.__vehicle_state = self.get_state()
        
        self.__pub_looptime = True
        self.__shutdown = False
        self.__finished = False
        self.__time_start = time.time()
        self.rpydot = np.zeros((1,3))

        self.__pos_cmd_timeout_factor = 1.0
        self.__pos_cmd_velocity_factor = 1.0/5.0
        self.__vel_cmd_timeout = 0.1 # SECONDS
        self.__service_timeout = 5.0 # SECONDS
        self.__max_velocity = 10.0 # m/s

        self.__wait_timeout = 5.0 # SECONDS

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
        rospy.Subscriber(cmd_pos_topic, PoseStamped, callback=self.__cmd_pos_cb, queue_size=10)


        self.__track_action_name = topic_prefix + "/track_object"
        self.__move_to_location_action_name = topic_prefix + "/move_to_location"

        self.__track_action = actionlib.SimpleActionServer(self.__track_action_name, TrackObjectAction, execute_cb=self.__track_action_cb, auto_start=False)
        self.__track_action.start()

        self.__move_to_location_action = actionlib.SimpleActionServer(self.__move_to_location_action_name, MoveToLocationAction, execute_cb=self.__move_to_location_action_cb, auto_start=False)
        self.__move_to_location_action.start()
        
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


    ##
    ##
    ###        ROS CALLBACKS            ###
    #######################################

    def __move_to_location_action_cb(self, goal):
        start_time = time.time()
        success = True

        target = airsim.Vector3r(goal.target[0], goal.target[1], goal.target[2])

        drone_pose = self.get_state()

        r = rospy.Rate(10)

        cmd = PoseStamped()
        cmd.header.frame_id = "global"

        feedback = MoveToLocationFeedback()
        
        drone_position = drone_pose.kinematics_estimated.position
        feedback.error = airsim.Vector3r.distance_to(target, drone_position)

        while time.time() - start_time < goal.timeout and feedback.error > goal.tolerance:
            if self.__move_to_location_action.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.__move_to_location_action_name)
                self.__move_to_location_action.set_preempted()
                success = False
                break             

            cmd.pose.position.x = target.x_val
            cmd.pose.position.y = target.y_val
            cmd.pose.position.z = target.z_val

            if(goal.yaw_frame == "local"):
                cmd.pose.orientation.z = goal.yaw + drone_pose.kinematics_estimated.orientation.z_val
            elif(goal.yaw_frame == "global"):
                cmd.pose.orientation.z = goal.yaw
            else:
                rospy.logerr("%s: Yaw frame_id unkown" % self.__move_to_location_action_name)
                success = False
                break

            self.__moveToPosition(cmd)


            drone_pose = self.get_state()
            drone_position = drone_pose.kinematics_estimated.position

            feedback.location = []
            feedback.location.append(drone_position.x_val)
            feedback.location.append(drone_position.y_val)
            feedback.location.append(drone_position.z_val)

            feedback.error = airsim.Vector3r.distance_to(target, drone_position)
            feedback.time_left = time.time() - start_time

            self.__move_to_location_action.publish_feedback(feedback)


            r.sleep()

        with self.__client_lock:
            self.__client.cancelLastTask(self.__drone_name)
            self.__client.hoverAsync(self.__drone_name)

        if success:
            result = feedback
            self.__move_to_location_action.set_succeeded(result)




    def __track_action_cb(self, goal):
        start_time = time.time()

        success = True

        with self.__client_lock:
            target_pose = self.__client.simGetObjectPose(goal.object_name)
        
        drone_pose = self.get_state()

        feedback_vector = calc_distance(target_pose.position, drone_pose.kinematics_estimated.position)
        feedback_magnitude = airsim.Vector3r.distance_to(drone_pose.kinematics_estimated.position, target_pose.position)

        r = rospy.Rate(10)
    
        cmd = PoseStamped()
        cmd.header.frame_id = "global"

        feedback = TrackObjectFeedback()

        print(self.__drone_name + " ENTERING WHILE LOOP")

        while time.time() - start_time < goal.timeout:
            if self.__track_action.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.__track_action_name)
                self.__track_action.set_preempted()
                success = False
                break
                
            cmd.pose.position.x = target_pose.position.x_val + goal.offset[0]
            cmd.pose.position.y = target_pose.position.y_val + goal.offset[1]
            cmd.pose.position.z = target_pose.position.z_val + goal.offset[2]

            self.__moveToPosition(cmd)


            with self.__client_lock:
                error_count = 0
                max_errors = 3
                error = Exception()

                for _ in range(0, max_errors):
                    try:
                        target_pose = self.__client.simGetObjectPose(goal.object_name)
                        break

                    except Exception as e:
                        error = e
                        error_count += 1

                if error_count == max_errors:
                    print(self.__drone_name + " Error from track object callback: {0}" .format(error.message))

            drone_pose = self.get_state()
            feedback_vector = calc_distance(target_pose.position, drone_pose.kinematics_estimated.position)
            feedback_magnitude = airsim.Vector3r.distance_to(drone_pose.kinematics_estimated.position, target_pose.position)
            
            feedback.dist = []
            feedback.dist.append(feedback_vector.x_val)
            feedback.dist.append(feedback_vector.y_val)
            feedback.dist.append(feedback_vector.z_val)
            feedback.dist_mag = feedback_magnitude

            self.__track_action.publish_feedback(feedback)


            r.sleep()

        with self.__client_lock:
            self.__client.cancelLastTask(self.__drone_name)
            self.__client.hoverAsync(self.__drone_name)

        if success:
            result = feedback
            self.__track_action.set_succeeded(result)


    def __cmd_pos_cb(self, msg):
        """
        Callback for the /cmd/pos ROS topic

        @param msg (geometry_msgs.PoseStamped)
        """

        with self.__flag_lock:
            self.__time_start = time.time()
            self.__finished = False      
            self.__cmd = msg

    def __moveToPosition(self, cmd):
        """
        Utilize AirSim's Python API to move the drone
        """

        with self.__flag_lock:
            if type(cmd) != PoseStamped:
                return False



            if cmd.header.frame_id == "local":
                orientation = self.__vehicle_state.kinematics_estimated.orientation
                position = self.__vehicle_state.kinematics_estimated.position

                (pitch, roll, yaw) = airsim.to_eularian_angles(orientation)
                
                # ASSUMING 0 PITCH AND ROLL
                x = cmd.pose.position.x*math.cos(yaw) - cmd.pose.position.y*math.sin(yaw) + position.x_val
                y = cmd.pose.position.x*math.sin(yaw) + cmd.pose.position.y*math.cos(yaw) + position.y_val
                z = cmd.pose.position.z + position.z_val
                yawmode = airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(cmd.pose.orientation.z+yaw))
            
            elif cmd.header.frame_id == "global":
                x = cmd.pose.position.x
                y = cmd.pose.position.y
                z = cmd.pose.position.z
                yawmode = airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(cmd.pose.orientation.z))

            else:
                print("DRONE " + self.__drone_name + " VEL cmd UNRECOGNIZED FRAME")
                return False

            goal = airsim.Vector3r(x, y, z)
            distance = airsim.Vector3r.distance_to(goal, self.__vehicle_state.kinematics_estimated.position)

            timeout = distance*self.__pos_cmd_timeout_factor*self.__pos_cmd_velocity_factor

            velocity = abs(distance*self.__pos_cmd_velocity_factor)
            velocity = min(velocity, self.__max_velocity)
            velocity = max(velocity, 1.0)

            with self.__client_lock:
                self.__client.moveToPositionAsync(x, y, z, velocity, timeout_sec=timeout, yaw_mode=yawmode, vehicle_name=self.__drone_name)





    def __cmd_vel_cb(self, msg):
        """
        Callback for the /cmd/vel ROS topic

        @param msg (geometry_msgs.TwistStamped)
        """

        with self.__flag_lock:
            self.__time_start = time.time()
            self.__finished = False      
            self.__cmd = msg


    def __moveAtVelocity(self, cmd):
        """
        Utilize AirSim's Python API to move the drone
        """

        with self.__flag_lock:
            if type(cmd) != TwistStamped:
                return False

            if cmd.header.frame_id == "local":
                orientation = self.__vehicle_state.kinematics_estimated.orientation

                #print(orientation)
                (pitch, roll, yaw) = airsim.to_eularian_angles(orientation)
                
                # ASSUMING 0 PITCH AND ROLL
                x = cmd.twist.linear.x*math.cos(yaw) - cmd.twist.linear.y*math.sin(yaw)
                y = cmd.twist.linear.x*math.sin(yaw) + cmd.twist.linear.y*math.cos(yaw)
                z = cmd.twist.linear.z
                yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(cmd.twist.angular.z))
            
            elif cmd.header.frame_id == "global":
                x = cmd.twist.linear.x
                y = cmd.twist.linear.y
                z = cmd.twist.linear.z
                yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(cmd.twist.angular.z))

            else:
                print("DRONE " + self.__drone_name + " VEL cmd UNRECOGNIZED FRAME")
                return False


            with self.__client_lock:
                self.__client.moveByVelocityAsync(x, y, z, duration=self.__vel_cmd_timeout, yaw_mode=yawmode, vehicle_name=self.__drone_name)


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
        with self.__flag_lock:
            self.__finished = False
            self.__shutdown = True

            with self.__client_lock:
                self.__client.armDisarm(False, vehicle_name=self.__drone_name)
                self.__client.enableApiControl(False, vehicle_name=self.__drone_name)

            self.__finished = True

            return SetBoolResponse(True, "")

    ##
    ##
    ###        LQR IMPLEMENATION        ###
    #######################################

    def testLQR(self, state, start_time, update_gains, plots):
        p = state.kinematics_estimated.position
        p = [p.x_val, p.y_val, p.z_val]

        q = state.kinematics_estimated.orientation
        q = [q.w_val, q.x_val, q.y_val, q.z_val]

        v = state.kinematics_estimated.linear_velocity
        v = [v.x_val, v.y_val, v.z_val]

        roll, pitch, yaw = lowlevel.LQR.quat2rpy(q)

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        tp = math.tan(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        R = np.matrix([ [1, sr*tp, cr*tp],
                        [0, cr, -sr],
                        [0, sr/cp, cr/cp]])

        x = lowlevel.LQR.set_state(p,q,v)
        x = lowlevel.LQR.ned2xyz(x)


        if update_gains:
            rpydot = state.kinematics_estimated.angular_velocity
            rpydot = np.array([[rpydot.y_val, -rpydot.x_val, rpydot.z_val]]).T
            omega = R.I*rpydot
            omega = [omega[0,0], omega[1,0], omega[2,0]]

            #c = (9.8-state.kinematics_estimated.linear_acceleration.z_val)/(cr*cp)
            # Compute thrust from command -> this is dangerous as there are unknown thrusts
            #   - start with open-loop from previous command
            #   - work to get motor thrust directly from AirSim
            #   - another option includes using nominal thrust, or 9.8/(cr*cp)

            c = (9.8-self.acceleration_cmds[-1,2])/(cr*cp)

            u = lowlevel.LQR.set_command(omega,c)

            self.__controller.updateGains(x,u)
            self.__prev_gain_time = time.time()



        u = self.__controller.computeControl(x)

        #print("command: " + str(u.T))

        #print("pos: " + str(p))
        #print("goal: " + str(p0))
        #print("vel: " + str(v))
        #print("orien: " + str([roll, pitch, yaw]))

        omega,c = lowlevel.LQR.get_command(u)

        tmp = omega[0,0]
        omega[0,0] = omega[1,0]
        omega[1,0] = tmp
        omega[2,0] = -omega[2,0]

        thrust = abs(c[0,0])*self.__mass

        for i in range(0, 3):
            if abs(omega[i]) > math.pi/2:
                print('WARNING -> RATE ' + str(i) + ' FOR ' + self.__drone_name + " GREATER THAN MAX RATE " + str(omega[i,0]))
                
            omega[i] = max(-math.pi/2, omega[i])
            omega[i] = min(math.pi/2, omega[i])


        rpydot = R*omega
        self.rpydot = np.append(self.rpydot, rpydot.T, axis=0)


        if(thrust > self.__max_thrust):
            print('WARNING -> THRUST FOR ' + self.__drone_name + ' GREATER THAN MAX THRUST ' + str(thrust))
            thrust = self.__max_thrust


        #t = time.time()-start_time
        #index = int(min(self.sim_time*self.freq - 1, math.ceil(t*self.freq)))
        #throttle = self.throttle_cmd[index] + 0.5863


        throttle = thrust / self.__max_thrust

        accelF = (throttle)*self.__max_thrust/self.__mass
        accel = np.zeros((3))

        accel[0] = (-cr*sp*cy - sr*sy)*accelF
        accel[1] = (-cr*sp*sy + sr*cy)*accelF
        accel[2] = 9.8-(cr*cp)*accelF
        
        self.acceleration_cmds = np.append(self.acceleration_cmds, [accel], axis=0)

        with self.__client_lock:
            # Calculate w/ equation 5 from submitted paper
            #omega_p = omega[0,0]
            #omega_q = omega[1,0]
            #omega_r = omega[2,0]
            roll_rate = rpydot[0,0]
            pitch_rate = rpydot[1,0]
            yaw_rate = rpydot[2,0]
            self.__client.moveByAngleRatesThrottleAsync(roll_rate, -pitch_rate, -yaw_rate, throttle, 0.5, self.__drone_name)   


        print("---")


    def run(self):
        self.__setup_ros()


        self.freq = 20
        self.sim_time = 15
        rate = rospy.Rate(self.freq)

        avg_time = 0

        i = 0

        #lowlevel.LowLevelController(self.__drone_name, self.__client_lock, self.__client)


        # Move to starting position (0,0,-5)
        with self.__client_lock:
            self.__client.moveToPositionAsync(0, 0, -15, 5).join()
            print("Reached (0,0,-15), starting motion")
            time.sleep(3)
            #self.__client.moveByAngleRatesThrottleAsync(math.pi/2, 0, 0, 1, 1, self.__drone_name)



        states = np.zeros((1,15))
        states[0,2] = -15

        times = np.zeros((1,1))
        self.acceleration_cmds = np.zeros((1,3))

        references = np.zeros((1,9))

        # Setup timing variables
        begin_time = time.time()
        prev_time = time.time()
        self.__prev_gain_time = begin_time
        prev_reference_time = 0
        update_gains = True


        # Calculate sinusoidal throttle test command
        t = np.linspace(0, self.sim_time, self.sim_time*self.freq)

        N = 20
        freqs = (np.random.random(N) - 0.5)*50

        sins = np.zeros((N,self.sim_time*self.freq))
        for i in range(0,N):
            sins[i] = np.sin(freqs[i]*t)

        self.throttle_cmd = 0.2*np.sum(sins,0)/N


        # Setup plots

        #plot1 = plt.figure(1)
        #plt.plot(t, -self.throttle_cmd*self.__max_thrust/self.__mass, '--g', label='ad0')
        
        plot2 = plt.figure(2)
        plt.grid()
        plots = [plt.subplot(221), plt.subplot(222), plt.subplot(223), plt.subplot(224)]


        # Setup initial reference position
        p0 = self.get_state().kinematics_estimated.position.to_numpy_array()



        # Main loop
        while time.time()-begin_time < self.sim_time:
            with self.__flag_lock:
                if self.__shutdown == True or rospy.is_shutdown():
                    break
                
                state = self.get_state()
                pos = state.kinematics_estimated.position.to_numpy_array()
                vel = state.kinematics_estimated.linear_velocity.to_numpy_array()

                q = state.kinematics_estimated.orientation
                q = np.array([q.w_val, q.x_val, q.y_val, q.z_val], dtype=np.float32)                
                r,p,y = lowlevel.LQR.quat2rpy(q)
                orien = np.array([r,p,y], dtype=np.float32)

                rpydot = state.kinematics_estimated.angular_velocity.to_numpy_array()
                accel = state.kinematics_estimated.linear_acceleration.to_numpy_array()
                
                statevector = np.concatenate((pos, vel, orien, rpydot, accel), axis=0)

                states = np.append(states, [statevector], axis=0)
                times = np.append(times, [[time.time()-begin_time]], axis=0)

                cmd = self.__cmd



                '''
                if(type(cmd) == TwistStamped):
                    self.__moveAtVelocity(cmd)

                elif(type(cmd) == PoseStamped):
                    self.__moveToPosition(cmd)

                self.__cmd = None

                '''
            if  np.linalg.norm(pos - np.array(p0)) < 0.25 and time.time() - prev_reference_time > 4: 
                p0[0] += 2
                p0[1] += 1
                p0[2] -= 1
                rpy = [0,0,0]
                q0 = lowlevel.LQR.rpy2quat(rpy[0], rpy[1], rpy[2])
                v0 = [0,0,0]

                x0 = lowlevel.LQR.set_state(p0, q0, v0)
                x0 = lowlevel.LQR.ned2xyz(x0)
            
                reference = np.concatenate((p0, v0, rpy), axis=0)

                omega = [0,0,0]
                c = 9.8
                u0 = lowlevel.LQR.set_command(omega, c)

                self.__controller.set_goals(x0, u0)
            
            if np.linalg.norm(pos - np.array(p0)) > 0.25:
                prev_reference_time = time.time()

            references = np.append(references, [reference], axis=0)

            # Update LQR gains at 10 Hz
            if time.time() - self.__prev_gain_time >= 0.01:
                update_gains = True

            # Compute the control at the main loop rate
            self.testLQR(state, begin_time, update_gains, plots)
            update_gains = False

            
            rate.sleep()

            # Calculate a publish looptime after sleeping
            if self.__pub_looptime:
                elapsed_time = time.time() - prev_time
                prev_time = time.time()

                msg = Float32(elapsed_time)
                self.__looptime_pub.publish(msg)
                print("%6.3f"% elapsed_time)


        print("Theoretical number of loops: " + str(self.throttle_cmd.shape))
        print("Actual number of loops: " + str(times.shape))



        plots[0].plot(times, states[:,0], 'b', label='n')
        plots[0].plot(times, states[:,1], 'r', label='e')
        plots[0].plot(times, states[:,2], 'g', label='d')
        plots[0].plot(times, references[:,0], '--b', label='n0')
        plots[0].plot(times, references[:,1], '--r', label='e0')
        plots[0].plot(times, references[:,2], '--g', label='d0')
        plots[0].set(xlabel='time', ylabel='m')
        plots[0].set_title('position')
        plots[0].legend(loc="upper right")

        plots[1].plot(times, states[:,3], 'b', label='vn')
        plots[1].plot(times, states[:,4], 'r', label='ve')
        plots[1].plot(times, states[:,5], 'g', label='vd')       
        plots[1].plot(times, references[:,3], '--b', label='vn0')
        plots[1].plot(times, references[:,4], '--r', label='ve0')
        plots[1].plot(times, references[:,5], '--g', label='vd0')        
        plots[1].set(xlabel='time', ylabel='m/s')
        plots[1].set_title('ned vecloties')
        plots[1].legend(loc="upper right")

        #plots[2].plot(times, states[:,6], color='blue', label='r')
        #plots[2].plot(times, states[:,7], color='red', label='p')
        #plots[2].plot(times, states[:,8], color='gray', label='y')
        #plots[2].set(xlabel='time', ylabel='rad')
        #plots[2].set_title('orientation')
        #plots[2].legend(loc="upper right")

        plots[2].plot(times, states[:,9], 'b', label='dr')
        plots[2].plot(times, states[:,10], 'r', label='dp')
        plots[2].plot(times, states[:,11], 'g', label='dy')
        plots[2].plot(times, self.rpydot[:,0], '--b', label='dr0')
        plots[2].plot(times, self.rpydot[:,1], '--r', label='dp0')
        plots[2].plot(times, self.rpydot[:,2], '--g', label='dy0')
        plots[2].set(xlabel='time', ylabel='rad/s')
        plots[2].set_title('angular velocity')
        plots[2].legend(loc="upper right")

        plots[3].set_title('angular velocity')
        plots[3].plot(times, states[:,12], 'b', label='an')
        plots[3].plot(times, states[:,13], 'r', label='ae')
        plots[3].plot(times, states[:,14], 'g', label='ad')
        plots[3].plot(times, self.acceleration_cmds[:,0], '--b', label='an0')
        plots[3].plot(times, self.acceleration_cmds[:,1], '--r', label='ae0')
        plots[3].plot(times, self.acceleration_cmds[:,2], '--g', label='ad0')
        plots[3].set(xlabel='time', ylabel='m/s^2')
        plots[3].set_title('linear acceleration')
        plots[3].legend(loc="upper right")

        plt.show()


        # Wait for last task to finish
        #with self.__client_lock:
        #    self.__client.cancelLastTask(vehicle_name=self.__drone_name)
        
        # Quit
        print(self.__drone_name + " QUITTING")
        time.sleep(0.5)




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