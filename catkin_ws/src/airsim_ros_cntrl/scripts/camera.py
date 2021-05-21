#! /usr/bin/python3

import rospy

from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import Takeoff, Land
import threading as th



class Camera(th.Thread):

    def __init__(self, drone_name: str):
        th.Thread.__init__(self)

        self.cameraPub = rospy.Publisher("/airsim_node/Camera/vel_cmd_body_frame", VelCmd, queue_size=5)
        self.cameraMsg = VelCmd()
        self.takeoff = rospy.ServiceProxy("/airsim_node/Camera/takeoff", Takeoff)
        self.land = rospy.ServiceProxy("/airsim_node/Camera/land", Land)


    def __publish_vel_cmd(self, msg: VelCmd, duration: float) -> None:
        r = rospy.Rate(10)
        t0 = rospy.get_time()

        while rospy.get_time() - t0 < duration:
            self.cameraPub.publish(msg)
            r.sleep()

    def moveAtVelAsync(self, msg: VelCmd, duration: float) -> th.Thread:
        t = th.Thread(target=self.__publish_vel_cmd, args=(msg, duration,))
        t.start()
        return t

    def takeOffAsync(self) -> th.Thread:
        t = th.Thread(target=self.takeoff.call, args=(True,))
        t.start()
        return t

    def landAsync(self):
        t = th.Thread(target=self.land.call, args=(True,))
        t.start()
        return t

    def run(self):
        vel = VelCmd()
        vel.twist.linear.z = -2
        vel.twist.linear.x = 0
        vel.twist.angular.z = 0

        self.moveAtVelAsync(vel, 16).join()

        vel.twist.linear.x = 1.5
        vel.twist.linear.z = 3
        vel.twist.angular.z = -0.05

        self.moveAtVelAsync(vel, 5).join()

        vel.twist.linear.z = 0
        vel.twist.angular.z = -0.01
        self.moveAtVelAsync(vel, 40).join()       


if __name__ == "__main__":

    pass