#! /usr/bin/python3

import threading

import rospy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import Takeoff, Land

class Drone:
  def __init__(self, swarmName, vehicleName):

    self.swarm_name = swarmName
    self.drone_name = vehicleName

    cmd_vel_body_topic = "/airsim_node/" + self.drone_name + "/vel_cmd_body_frame"
    cmd_vel_world_topic = "/airsim_node/" + self.drone_name + "/vel_cmd_world_frame"

    self.cmd_vel_body_pub = rospy.Publisher(cmd_vel_body_topic, VelCmd, queue_size=10)
    self.cmd_vel_world_pub  = rospy.Publisher(cmd_vel_world_topic, VelCmd, queue_size=10)

    self.vel_msg = VelCmd()
    self.vel_frame = "body"

    self._lock = threading.Lock()
    self._shutdown = False
    self._publish_vel_cmd = False


  def set_velocity_local(self, cmd):
    with self._lock:
      self.vel_msg.twist.linear.x = cmd["lx"]
      self.vel_msg.twist.linear.y = cmd["ly"]
      self.vel_msg.twist.linear.z = cmd["lz"]
      self.vel_msg.twist.angular.x = cmd["ax"]
      self.vel_msg.twist.angular.y = cmd["ay"]
      self.vel_msg.twist.angular.z = cmd["az"]

      self.vel_frame = "body"
      self._publish_vel_cmd = True

  def set_velocity_world(self, cmd):
    with self._lock:
      self.vel_msg.twist.linear.x = cmd["lx"]
      self.vel_msg.twist.linear.y = cmd["ly"]
      self.vel_msg.twist.linear.z = cmd["lz"]
      self.vel_msg.twist.angular.x = cmd["ax"]
      self.vel_msg.twist.angular.y = cmd["ay"]
      self.vel_msg.twist.angular.z = cmd["az"]

      self.vel_frame = "world"
      self._publish_vel_cmd = True

  def hover(self):
    with self._lock:
      self._publish_vel_cmd = False


  def takeoff(self, wait=False):
    with self._lock:
      service_name = "/airsim_node/" + self.drone_name + "/takeoff"
      rospy.wait_for_service(service_name)

      try:
        takeoff = rospy.ServiceProxy(service_name, Takeoff)
        resp = takeoff(wait)
        return resp
      except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


  def land(self, wait=False):
    with self._lock:
      service_name = "/airsim_node/" + self.drone_name + "/land"
      rospy.wait_for_service(service_name)

      try:
        land = rospy.ServiceProxy(service_name, Land)
        resp = land(wait)
        return resp
      except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

  def shutdown(self, shutdown=True):
    with self._lock:
      self.vel_msg.twist.linear.x = 0
      self.vel_msg.twist.linear.y = 0
      self.vel_msg.twist.linear.z = 0
      self.vel_msg.twist.angular.x = 0
      self.vel_msg.twist.angular.y = 0
      self.vel_msg.twist.angular.z = 0

      self.cmd_vel_body_pub.publish(self.vel_msg)

      self._shutdown = True


  def fly(self, rate):
    r = rospy.Rate(rate)
    i = 0
    while not rospy.is_shutdown() and self._shutdown == False:

      if(self._publish_vel_cmd == True):
        if(self.vel_frame == "world"):
          self.cmd_vel_world_pub.publish(self.vel_msg)
        elif(self.vel_frame == "body"):
          self.cmd_vel_body_pub.publish(self.vel_msg)

      r.sleep()
