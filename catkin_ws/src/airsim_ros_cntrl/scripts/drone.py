#! /usr/bin/python3

import threading
import math

import airsim

#import rospy
#from airsim_ros_pkgs.msg import VelCmd
#from airsim_ros_pkgs.srv import Takeoff, Land

#from nav_msgs.msg import Odometry
#from sensor_msgs.msg import NavSatFix, Imu

class Drone:
  def __init__(self, swarmName, vehicleName):

    self.swarm_name = swarmName
    self.drone_name = vehicleName

    self.client = airsim.MultirotorClient()
    self.client.confirmConnection()
    self.client.enableApiControl(True, vehicle_name=self.drone_name)
    self.client.armDisarm(True, vehicle_name=self.drone_name)

    self.state = self.get_state()

    '''
    cmd_vel_body_topic  = "/airsim_node/" + self.drone_name + "/vel_cmd_body_frame"
    cmd_vel_world_topic = "/airsim_node/" + self.drone_name + "/vel_cmd_world_frame"

    odom_topic = "/airsim_node/" + self.drone_name + "/odom_local_ned"
    gps_topic  = "/airsim_node/" + self.drone_name + "/global_gps"
    imu_topic  = "/airsim_node/" + self.drone_name + "/imu/imu_1"

    self.cmd_vel_body_pub   = rospy.Publisher(cmd_vel_body_topic, VelCmd, queue_size=10)
    self.cmd_vel_world_pub  = rospy.Publisher(cmd_vel_world_topic, VelCmd, queue_size=10)

    rospy.Subscriber(odom_topic, Odometry, callback=self.odom_cb, queue_size=10)
    rospy.Subscriber(gps_topic, NavSatFix, callback=self.gps_cb, queue_size=10)
    rospy.Subscriber(imu_topic, Imu, callback=self.imu_cb, queue_size=10)
    '''
    self.vel_cmd = dict([('x',0), ('y',0), ('z',0), ('dur',0), ('yawmode',airsim.YawMode())])
    self.vel_frame = "body"
    
    self._lock = threading.Lock()
    self._shutdown = False
    self._publish_vel_cmd = False
  

  '''
  def odom_cb(self, msg):
    test = 1
    #print(self.drone_name + " Odom: %d" %msg.pose.pose.position.x)

  def gps_cb(self, msg):
    test = 1 
    #print(self.drone_name + " GPS: %d" %msg.altitude)
  
  def imu_cb(self, msg):
    test =1 
    #print(self.drone_name + " Z accel: %d" %msg.linear_acceleration.z)
  '''
  def get_client(self):
    return self.client

  def get_name(self):
    return self.drone_name

  def get_state(self):
    self.state = self.client.getMultirotorState(vehicle_name=self.drone_name)
    return self.client.getMultirotorState(vehicle_name=self.drone_name)


  def movetoPosition(self, cmd, wait=False):
    if wait:
      self.client.moveToPositionAsync(cmd['x'], cmd['y'], cmd['z'], cmd['vel'], cmd['timeout'], vehicle_name=self.drone_name).join()
    else:
      self.client.moveToPositionAsync(vehicle_name=self.drone_name)
  
  def set_velocity_local(self, cmd):    
    with self._lock:
      (pitch, roll, yaw) = airsim.to_eularian_angles(self.get_state().kinematics_estimated.orientation)
      #yaw = math.radians(yaw)
      self.vel_cmd['x'] = cmd['lx']*math.sin(yaw) - cmd['ly']*math.sin(yaw)
      self.vel_cmd['y'] = cmd['lx']*math.cos(yaw) + cmd['ly']*math.cos(yaw)
      self.vel_cmd['z'] = cmd['lz']

      self.vel_cmd['yawmode'] = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(cmd['az']))
      self._publish_vel_cmd = True
    #self.client.moveByVelocityAsync(x, y, z, cmd['dur'], yaw_mode=yawmode, vehicle_name=self.drone_name)

    '''
    self.client.moveByVelocityAsync(cmd["lx"], cmd["ly"], cmd["lz"], duration=cmd["dur"], yaw_mode=)
    
    with self._lock:
      self.vel_msg.twist.linear.x = cmd["lx"]
      self.vel_msg.twist.linear.y = cmd["ly"]
      self.vel_msg.twist.linear.z = cmd["lz"]
      self.vel_msg.twist.angular.x = cmd["ax"]
      self.vel_msg.twist.angular.y = cmd["ay"]
      self.vel_msg.twist.angular.z = cmd["az"]

      self.vel_frame = "body"
      self._publish_vel_cmd = True
      '''
    
  
  def set_velocity_world(self, cmd):
    with self._lock:
      self.vel_cmd['x'] = cmd['lx']
      self.vel_cmd['y'] = cmd['ly']
      self.vel_cmd['z'] = cmd['lz']
      self.vel_cmd['yawmode'] = airsim.YawMode(is_rate=True, yaw_or_rate=cmd['az'])
      self._publish_vel_cmd = True
    #self.client.moveByVelocityAsync(cmd['lx'], cmd['ly'], cmd['lz'], cmd['dur'], yaw_mode=yawmode, vehicle_name=self.drone_name)

    '''
    with self._lock:
      self.vel_msg.twist.linear.x = cmd["lx"]
      self.vel_msg.twist.linear.y = cmd["ly"]
      self.vel_msg.twist.linear.z = cmd["lz"]
      self.vel_msg.twist.angular.x = cmd["ax"]
      self.vel_msg.twist.angular.y = cmd["ay"]
      self.vel_msg.twist.angular.z = cmd["az"]

      self.vel_frame = "world"
      self._publish_vel_cmd = True
    '''

  def hover(self, wait=False):
    with self._lock:
      self._publish_vel_cmd = False

    if wait:
      self.client.hoverAsync(vehicle_name=self.drone_name).join()
    else:
      self.client.hoverAsync(vehicle_name=self.drone_name)

    #with self._lock:
    #  self._publish_vel_cmd = False


  def takeoff(self, wait=False):
    if wait:
      self.client.takeoffAsync(vehicle_name=self.drone_name).join()
    else:
      self.client.takeoffAsync(vehicle_name=self.drone_name)
      
    '''
    with self._lock:
      service_name = "/airsim_node/" + self.drone_name + "/takeoff"
      rospy.wait_for_service(service_name)

      try:
        takeoff = rospy.ServiceProxy(service_name, Takeoff)
        resp = takeoff(wait)
        return resp
      except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    '''

  def land(self, wait=False):
    if wait:
      self.client.landAsync(vehicle_name=self.drone_name).join()
    else:
      self.client.landAsync(vehicle_name=self.drone_name)

    '''
    with self._lock:
      service_name = "/airsim_node/" + self.drone_name + "/land"
      rospy.wait_for_service(service_name)

      try:
        land = rospy.ServiceProxy(service_name, Land)
        resp = land(wait)
        return resp
      except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    '''

  def shutdown(self, shutdown=True):
    self.hover()

    self.client.armDisarm(False, vehicle_name=self.drone_name)
    self.client.enableApiControl(is_enabled=False, vehicle_name=self.drone_name)

    '''
    with self._lock:
      self.vel_msg.twist.linear.x = 0
      self.vel_msg.twist.linear.y = 0
      self.vel_msg.twist.linear.z = 0
      self.vel_msg.twist.angular.x = 0
      self.vel_msg.twist.angular.y = 0
      self.vel_msg.twist.angular.z = 0

      self.cmd_vel_body_pub.publish(self.vel_msg)

      self._shutdown = True
    '''


  def fly(self, rate):

    while(self._shutdown == False):
      if(self._publish_vel_cmd == True):
        #print(self.vel_cmd['x'])
        self.client.moveByVelocityAsync(self.vel_cmd['x'], self.vel_cmd['y'], self.vel_cmd['z'], 1, self.vel_cmd['yawmode'], vehicle_name=self.drone_name)

    '''
    r = rospy.Rate(rate)
    while not rospy.is_shutdown() and self._shutdown == False:

      if(self._publish_vel_cmd == True):
        if(self.vel_frame == "world"):
          self.cmd_vel_world_pub.publish(self.vel_msg)
        elif(self.vel_frame == "body"):
          self.cmd_vel_body_pub.publish(self.vel_msg)

      r.sleep()
    '''