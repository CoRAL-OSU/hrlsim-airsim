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

            x_y_gains = airsim.PIDGains(0.2, 0.0, 0.05)
            z_gains = airsim.PIDGains(4.0, 2.0, 0.0)
            vel_pid_gains = airsim.VelocityControllerGains(x_gains=x_y_gains, y_gains=x_y_gains, z_gains=z_gains)

            #self.__client.setVelocityControllerGains(velocity_gains=vel_pid_gains, vehicle_name=self.__drone_name)

        self.__vehicle_state = self.get_state()
        
        self.__pub_looptime = True
        self.__shutdown = False
        self.__finished = False
        self.__time_start = time.time()

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


    def testLQR(self, state, start_time, update_gains, plots):
        if update_gains:
            omega = state.kinematics_estimated.angular_velocity
            omega = [omega.y_val, omega.x_val, omega.z_val]
            c = state.kinematics_estimated.linear_acceleration.z_val

            u = lowlevel.LQR.set_command(omega,c)

            p = state.kinematics_estimated.position
            p = [p.x_val, p.y_val, p.z_val]

            q = state.kinematics_estimated.orientation
            q = [q.w_val, q.x_val, q.y_val, q.z_val]

            v = state.kinematics_estimated.linear_velocity
            v = [v.x_val, v.y_val, v.z_val]

            x = lowlevel.LQR.set_state(p,q,v)
            #x = lowlevel.LQR.ned2xyz(x)

            self.__controller.updateGains(x,u)
            self.__prev_gain_time = time.time()

        p = state.kinematics_estimated.position
        p = [p.x_val, p.y_val, p.z_val]

        q = state.kinematics_estimated.orientation
        q = [q.w_val, q.x_val, q.y_val, q.z_val]

        v = state.kinematics_estimated.linear_velocity
        v = [v.x_val, v.y_val, v.z_val]

        roll, pitch, yaw = lowlevel.LQR.quat2rpy(q)

        x = lowlevel.LQR.set_state(p,q,v)
        #x = lowlevel.LQR.ned2xyz(x)

        #t = time.time()
        #plots[0].scatter(t-start_time, roll, 2, marker='o', color='blue', label='roll')
        #plots[0].scatter(t-start_time, pitch, 2, marker='o', color='red', label='pitch')
        #plots[0].scatter(t-start_time, yaw, 2, marker='o', color='gray', label='yaw')

        print("angular vel: " + str(state.kinematics_estimated.angular_velocity.to_numpy_array()))


        u = self.__controller.computeControl(x)

        #! Add plots
        #! Plot step changes in vertical direction with LQR

        #u = lowlevel.LQR.set_command(omega,c)

        print("command: " + str(u.T))

        print("pos: " + str(p))
        #print("goal: " + str(p0))
        print("vel: " + str(v))
        print("orien: " + str([roll, pitch, yaw]))

        omega,c = lowlevel.LQR.get_command(u)

        thrust = -c[0,0]*self.__mass

        for i in range(0, 3):
            if abs(omega[i]) > math.pi/2:
                print('WARNING -> RATE ' + str(i) + ' FOR ' + self.__drone_name + " GREATER THAN MAX RATE " + str(omega[i,0]))
                
            omega[i] = max(-math.pi/2, omega[i])
            omega[i] = min(math.pi/2, omega[i])


        if(thrust > self.__max_thrust):
            print('WARNING -> THRUST FOR ' + self.__drone_name + ' GREATER THAN MAX THRUST ' + str(thrust))
            thrust = self.__max_thrust


        throttle = thrust / self.__max_thrust

        with self.__client_lock:
            # Calculate w/ equation 5 from submitted paper
            roll_rate = 0# omega[0,0] + math.sin(roll)*math.tan(pitch)*omega[1,0] + math.cos(roll)*math.tan(pitch)*omega[2,0] # math.pi/20      phi dot
            pitch_rate = math.cos(roll)*omega[1,0] - math.sin(roll)*omega[2,0] # math.pi/20     theta dot
            yaw_rate = 0   # math.sin(roll)/math.cos(pitch)*omega[1,0] + math.cos(roll)/math.cos(pitch)*omeag[2,0]                   psi dot

            print("cmd rates: " + str([roll_rate,pitch_rate,yaw_rate]))

            self.__client.moveByAngleRatesThrottleAsync(roll_rate, pitch_rate, yaw_rate, throttle, 1.0, self.__drone_name)            


        print("---")


    def run(self):
        self.__setup_ros()

        freq = 20
        rate = rospy.Rate(freq)

        avg_time = 0

        i = 0

        #lowlevel.LowLevelController(self.__drone_name, self.__client_lock, self.__client)


        # Move to starting position (0,0,-5)
        with self.__client_lock:
            self.__client.moveToPositionAsync(0, 0, -5, 5).join()
            print("Reached (0,0,-5), starting motion")
            time.sleep(1)
            #self.__client.moveByAngleRatesThrottleAsync(math.pi/2, 0, 0, 1, 1, self.__drone_name)


        # Set up goal position
        p0 = [1,0,-5]
        q0 = [1,0,0,0]
        v0 = [0,0,0]
        x0 = lowlevel.LQR.set_state(p0, q0, v0)
        #x0 = lowlevel.LQR.ned2xyz(x0)

        omega = [0,0,0]
        c = 9.8
        u0 = lowlevel.LQR.set_command(omega, c)

        self.__controller.set_goals(x0, u0)

        # Setup timing variables
        begin_time = time.time()
        prev_time = time.time()
        self.__prev_gain_time = begin_time
        update_gains = True

        states = list()
        times = list()

        plt.grid()
        plots = [plt.subplot(221), plt.subplot(222), plt.subplot(223), plt.subplot(224)]


        # Main loop
        while time.time()-begin_time < 5:
            with self.__flag_lock:
                if self.__shutdown == True or rospy.is_shutdown():
                    break
                
                state = self.get_state()
                states.append(state)
                times.append(time.time())
                cmd = self.__cmd

            '''
            if(type(cmd) == TwistStamped):
                self.__moveAtVelocity(cmd)

            elif(type(cmd) == PoseStamped):
                self.__moveToPosition(cmd)

            self.__cmd = None

            '''

            # Update LQR gains at 10 Hz
            if time.time() - self.__prev_gain_time >= 0.1:
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
                print(elapsed_time)

        print(states[:])
        plots[0].scatter(times, states[:].kinematics_estimated.angular_velocity.x, 2, marker='o', color='blue', label='roll')
        #plots[0].scatter(times, pitch, 2, marker='o', color='red', label='pitch')
        #plots[0].scatter(times, yaw, 2, marker='o', color='gray', label='yaw')

        plots[0].set(xlabel='time', ylabel='radians/s')
        plots[0].set_title('roll/pitch/yaw rates')
        plots[0].legend(loc="upper right")
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