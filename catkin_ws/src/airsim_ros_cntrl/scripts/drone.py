#! /usr/bin/python3

import time
import threading
import math

import airsim


class Drone:
  def __init__(self, swarmName, vehicleName, sim_client, threading_lock):

    self.swarm_name = swarmName
    self.drone_name = vehicleName

    self._lock = threading_lock

    with self._lock:
      self.client = sim_client
      self.client.confirmConnection()
      self.client.enableApiControl(True, vehicle_name=self.drone_name)
      self.client.armDisarm(True, vehicle_name=self.drone_name)

    self.state = self.get_state()

    self.vel_cmd = dict([('x',0), ('y',0), ('z',0), ('dur',0), ('yawmode',airsim.YawMode())])
    self.vel_frame = "body"
    
    self._shutdown = False
    self._publish_vel_cmd = False
  

  def get_client(self):
    return self.client

  def get_name(self):
    return self.drone_name

  def get_state(self):
    with self._lock:
      self.state = self.client.getMultirotorState(vehicle_name=self.drone_name)
    return self.state


  def movetoPosition(self, cmd, wait=False):
    with self._lock:
      if wait:
        self.client.moveToPositionAsync(cmd['x'], cmd['y'], cmd['z'], cmd['vel'], cmd['timeout'], vehicle_name=self.drone_name).join()
      else:
        self.client.moveToPositionAsync(cmd['x'], cmd['y'], cmd['z'], cmd['vel'], cmd['timeout'], vehicle_name=self.drone_name)
  
  def set_velocity(self, cmd):    
    time_start = time.time()
    
    if cmd["frame"] == "body": 
      while(time.time() - time_start <= cmd['dur']):
        (pitch, roll, yaw) = airsim.to_eularian_angles(self.get_state().kinematics_estimated.orientation)

        
        x = cmd['lx']*math.cos(yaw) - cmd['ly']*math.sin(yaw)
        y = cmd['lx']*math.sin(yaw) + cmd['ly']*math.cos(yaw)
        z = cmd['lz']
        yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(cmd['az']))

        #print("DRONE: " + self.drone_name + " ATTEMPTING LOCK")
        with self._lock:
          #print("DRONE: " + self.drone_name + "     LOCK AQUIRED    yaw: %f" %yaw)
          self.client.moveByVelocityAsync(x, y, z, 0.1, yaw_mode=yawmode, vehicle_name=self.drone_name)

        time.sleep(0.05)

    elif cmd["frame"] == "world":
      yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=cmd['az'])

      with self._lock:
        self.client.moveByVelocityAsync(cmd['lx'], cmd['ly'], cmd['lz'], cmd['dur'], yaw_mode=yawmode, vehicle_name=self.drone_name)     


    else:
      print("DRONE " + self.drone_name + " UNRECOGNIZED FRAME")


  def hover(self, wait=False):
    self._publish_vel_cmd = False

    with self._lock:
      if wait:
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
      else:
        self.client.hoverAsync(vehicle_name=self.drone_name)


  def takeoff(self, wait=False):
    if self.get_state().landed_state == airsim.LandedState.Flying:
      return

    with self._lock:
      if wait:
        self.client.takeoffAsync(vehicle_name=self.drone_name).join()
      else:
        self.client.takeoffAsync(vehicle_name=self.drone_name)
      

  def land(self, wait=False):
    if self.get_state().landed_state == airsim.LandedState.Landed:
      return
    
    with self._lock:
      if wait:
        self.client.landAsync(vehicle_name=self.drone_name).join()
      else:
        self.client.landAsync(vehicle_name=self.drone_name)


  def shutdown(self, shutdown=True):
    self.hover()

    with self._lock:
      self.client.armDisarm(False, vehicle_name=self.drone_name)
      self.client.enableApiControl(is_enabled=False, vehicle_name=self.drone_name)




'''
  def fly(self, rate):

    while(self._shutdown == False):
      if(self._publish_vel_cmd == True):
        print(self.vel_cmd['x'])
        with self._lock:
          self.client.moveByVelocityAsync(self.vel_cmd['x'], self.vel_cmd['y'], self.vel_cmd['z'], 1, self.vel_cmd['yawmode'], vehicle_name=self.drone_name)
'''