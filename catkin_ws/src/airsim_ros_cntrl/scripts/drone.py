#! /usr/bin/python3

import rospy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import Takeoff, Land

class Drone:
  def __init__(self, vehicleName):

    self.name = vehicleName

    cmd_vel_body_topic = "/airsim_node/" + self.name + "/vel_cmd_body_frame"
    cmd_vel_world_topic = "/airsim_node/" + self.name + "/vel_cmd_world_frame"

    self.cmd_vel_body_pub = rospy.Publisher(cmd_vel_body_topic, VelCmd, queue_size=10)
    self.cmd_vel_world_pub  = rospy.Publisher(cmd_vel_world_topic, VelCmd, queue_size=10)

  def set_velocity_local(self, cmd):
    msg = VelCmd()
    msg.twist.linear.x = cmd["lx"]
    msg.twist.linear.y = cmd["ly"]
    msg.twist.linear.z = cmd["lz"]
    msg.twist.angular.x = cmd["ax"]
    msg.twist.angular.y = cmd["ay"]
    msg.twist.angular.z = cmd["az"]

    self.cmd_vel_body_pub.publish(msg)


  def set_velocity_world(self, cmd):
    msg = VelCmd()
    msg.twist.linear.x = cmd["lx"]
    msg.twist.linear.y = cmd["ly"]
    msg.twist.linear.z = cmd["lz"]
    msg.twist.angular.x = cmd["ax"]
    msg.twist.angular.y = cmd["ay"]
    msg.twist.angular.z = cmd["az"]

    self.cmd_vel_world_pub.publish(msg)


  def takeoff(self, wait=False):
    service_name = "/airsim_node/" + self.name + "/takeoff"
    rospy.wait_for_service(service_name)

    try:
      takeoff = rospy.ServiceProxy(service_name, Takeoff)
      resp = takeoff(wait)
      return resp
    except rospy.ServiceException as e:
      print("Service call failed: %s" %e)


  def land(self, wait=False):
    service_name = "/airsim_node/" + self.name + "/land"
    rospy.wait_for_service(service_name)

    try:
      land = rospy.ServiceProxy(service_name, Land)
      resp = land(wait)
      return resp
    except rospy.ServiceException as e:
      print("Service call failed: %s" %e)


