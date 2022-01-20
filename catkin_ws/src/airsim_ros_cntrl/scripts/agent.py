#! /usr/bin/python3

import rospy, actionlib, sys

import numpy as np

import drone

from airsim.types import MultirotorState, Vector3r
from airsim_ros_cntrl.msg import (
    Multirotor,
    TrackObjectAction,
    TrackObjectFeedback,
    TrackObjectResult,
    TrackObjectGoal,
    MoveToLocationGoal,
)

from airsim_ros_pkgs.msg import GimbalAngleEulerCmd, GimbalAngleQuatCmd

class Agent(drone.Drone):
    """
    Class to handle individual agents of a team.
    An agent is a drone that takes commands to either track an object or move to a certain location.
    Refer to drone.Drone documentation for more info.

    Args:
        swarmName (str): The name of the team this drone is associated with.
        droneName (str): The name of the drone itself.
    """

    def __init__(
        self,
        swarmName: str,
        droneName: str
    ) -> None:
        """
        Constructs a new Agent Process.

        Args:
            swarmName (str): The name of the swarm this drone is associated with.
            droneName (str): The name of the drone itself.
        """
        super().__init__(swarmName, droneName)

        self.target = None

        self.__target_pose = MultirotorState()
        self.__target_ready = False

        print("NEW AGENT: " + self.drone_name)


    def setup_ros(self) -> None:
        """
        @Override
        Sets up the ros node for use during simulation.
        Must be called during the run function since the handoff is not done from multiprocessing until after started.

        Topics:

        Services:
            /takeoff (airsim_ros_pkgs.Takeoff): Service to takeoff the drone
            /land (airsim_ros_pkgs.Land): Service to land the drone
            /shutdown (std_srvs.SetBool): Service to shutdown process
        
        Actions:
            /track_object (airsim_ros_cntrl.TrackObjectAction): Action to track a target.

        Returns: None
        """
        drone.Drone.setup_ros(self)

        prefix = self.swarm_name + "/" + self.drone_name + "/"

        self.__gimbal_pub = rospy.Publisher("/airsim_node/gimbal_angle_quat_cmd", GimbalAngleQuatCmd, queue_size=2)
        self.__track_action_name = prefix + "track_object"

        self.__track_action = actionlib.SimpleActionServer(
            self.__track_action_name,
            TrackObjectAction,
            execute_cb=self.__track_action_cb,
            auto_start=False,
        )
        self.__track_action.start()

    ##
    ##
    ###        ROS CALLBACKS            ###
    #######################################

    def __target_state_cb(self, msg: Multirotor) -> None:
        """
        Subscriber callback for tracking target position.
        Used in /track_object

        Args:
            msg (airsim_ros_cntrl/Multirotor)

        Returns: None
        """
        pos = msg.state.pose.position
        self.__target_pose.kinematics_estimated.position = Vector3r(
            pos.x, pos.y, pos.z
        )
    
        vel = msg.state.vel
        self.__target_pose.kinematics_estimated.linear_velocity = Vector3r(
            vel.linear.x, vel.linear.y, vel.linear.z
        )   
        self.__target_pose.kinematics_estimated.angular_velocity = Vector3r(
            vel.angular.x, vel.angular.y, vel.angular.z
        )

        acc = msg.state.acc
        self.__target_pose.kinematics_estimated.linear_acceleration = Vector3r(
            acc.linear.x, acc.linear.y, acc.linear.z
        )
        self.__target_pose.kinematics_estimated.angular_acceleration = Vector3r(
            acc.angular.x, acc.angular.y, acc.angular.z
        )      


        self.__target_ready = True



    def __track_action_cb(self, goal: TrackObjectGoal) -> TrackObjectResult:
        """
        Callback to track a target.
        Subscribes to the position, velocity and acceleration,
        then tracks the object using LQR to control with minimum snap.

        Args:
            goal (TrackObjectGoal): The object to track

        Topic:
            /track_object
        
        Feedback (TrackObjectFeedback): Continuous feedback about how tracking is going

        Returns (TrackObjectResult): The Result after tracking
        """
        start_time = rospy.get_time()

        success = True
        target_topic = self.swarm_name + "/" + goal.object_name
        print(target_topic)

        target_state_sub = rospy.Subscriber(
            target_topic,
            Multirotor,
            callback=self.__target_state_cb,
            queue_size=10,
        )

        while self.__target_ready == False and rospy.get_time() - start_time < goal.timeout:
            rospy.sleep(0.05)

        sleeper = rospy.Rate(self.freq)


        feedback = TrackObjectFeedback()
        print(self.drone_name + " ENTERING WHILE LOOP")

        update_object_location_period = 1.0  # seconds
        prev_object_update_time = 0

        if goal.timeout > 0:
            runforever = False
        else:
            runforever = True


        first = True
        while runforever == True or rospy.get_time() - start_time < goal.timeout:
            if self.__track_action.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self.__track_action_name)
                self.__track_action.set_preempted()
                success = False
                break

            p = self.state.kinematics_estimated.position.to_numpy_array()
            target_pose = self.__target_pose.kinematics_estimated
            tp = target_pose.position.to_numpy_array()

            if rospy.get_time() - prev_object_update_time > update_object_location_period:

                target_vel = target_pose.linear_velocity.to_numpy_array()
                target_acc = target_pose.linear_acceleration.to_numpy_array()

                bias = target_vel*update_object_location_period + 0.5*target_acc*update_object_location_period**2

                pt = tp + goal.offset + bias

                waypoints = np.array([p,pt]).T

                fa = target_acc
                fv = target_vel + target_acc*update_object_location_period
                fj = np.zeros(3)
                fc = np.array([fv,fa,fj])

                ia = self.state.kinematics_estimated.linear_acceleration.to_numpy_array()
                iv = self.state.kinematics_estimated.linear_velocity.to_numpy_array()
                ij = np.zeros(3)
                ic = np.array([iv, ia, ij])

                d = np.linalg.norm((p-(tp+goal.offset)))

                target_spd = np.linalg.norm(target_vel)

                if d < 0.25:
                    spd_gain = 0
                else:
                    spd_gain = 0.5
                
                avg_spd = target_spd + spd_gain*d
                avg_spd = np.minimum(avg_spd, 3.0)
                avg_spd = np.maximum(avg_spd, 0.1)

                if target_spd != 0 or first:
                    self.controller.set_goals(waypoints, ic, fc, avg_spd)
                    prev_object_update_time = rospy.get_time()
                    self.t0 = prev_object_update_time
                    first = False

                cmd = MoveToLocationGoal()
                cmd.target = pt
                cmd.timeout = 1/self.freq * 2
                cmd.tolerance = 0.1
                cmd.speed = avg_spd

                cmd.fvel = fv
                cmd.facc = fa
                cmd.fjrk = fj

                cmd.yaw_frame = "world"
                cmd.yaw = 0

                self.cmd = cmd

            feedback_vector = Vector3r(*(tp+goal.offset)) - self.state.kinematics_estimated.position

            feedback.dist = []
            feedback.dist.append(feedback_vector.x_val)
            feedback.dist.append(feedback_vector.y_val)
            feedback.dist.append(feedback_vector.z_val)
            feedback.dist_mag = feedback_vector.get_length()


            '''
            feedback_dir = feedback_vector/feedback_vector.get_length()

            normpos = self.state.kinematics_estimated.position/self.state.kinematics_estimated.position.get_length()
            normtar = target_pose.position/target_pose.position.get_length()

            gimbal_orien = target_pose.position.cross(self.state.kinematics_estimated.position).to_Quaternionr()
            gimbal_orien.w_val = math.sqrt((target_pose.position.get_length()**2)*(self.state.kinematics_estimated.position.get_length()**2)) +  math.acos(normtar.dot(normpos))
            gimbal_orien = gimbal_orien / gimbal_orien.get_length()

            (gp, gr, gy) = airsim.to_eularian_angles(gimbal_orien)
            (p,r,y) = airsim.to_eularian_angles(self.state.kinematics_estimated.orientation)


            gimbal_msg = GimbalAngleQuatCmd()
            #gimbal_msg.orientation = Quaternion(*gimbal_orien.to_numpy_array())
            gimbal_msg.camera_name = "cam0"
            gimbal_msg.vehicle_name = self.drone_name

            #self.__gimbal_pub.publish(gimbal_msg)
            '''

            self.__track_action.publish_feedback(feedback)
            sleeper.sleep()
            
        self.cmd = None


        self.__target_ready = False
        target_state_sub.unregister()


        if success:
            result = feedback
            self.__track_action.set_succeeded(result)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])

    agent = Agent("swarm", drone_name)
    agent.start()
    agent.join()
