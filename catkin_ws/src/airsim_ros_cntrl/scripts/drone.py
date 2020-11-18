#! /usr/bin/python3

import time
import threading
import math

import airsim


def makePosCmd(drone=None, timeout=3e38, frame="world", x=0, y=0, z=0, yaw=0, vel=0):
    return((dict([('drone',drone), ('frame',frame), ('timeout',timeout), ('x',x), ('y',y), ('z',z), ('yaw',yaw), ('vel',vel)])))

def makeVelCmd(drone=None, dur=0.01, frame="body", lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
    return((dict([('drone',drone), ('frame',frame), ('dur',dur), ('lx',lx), ('ly',ly), ('lz',lz), ('ax', ax), ('ay',ay), ('az',az)])))


class Drone:
  def __init__(self, swarmName, vehicleName, sim_client, threading_lock):
    self.swarm_name = swarmName
    self.drone_name = vehicleName

    self.__client_lock = threading_lock
    self.__threading_lock = threading.Lock()

    with self.__client_lock:
      self.client = sim_client
      self.client.confirmConnection()
      self.client.enableApiControl(True, vehicle_name=self.drone_name)
      self.client.armDisarm(True, vehicle_name=self.drone_name)

    self.state = self.get_state()

    self.__cmd_thread = None
    
    self.__shutdown = False
    self.__finished = False
    self.__time_start = time.time()
    self.__cmd_timeout = 1  

  def get_client(self):
    return self.client

  def get_name(self):
    return self.drone_name

  def get_state(self):
    with self.__client_lock:
      self.state = self.client.getMultirotorState(vehicle_name=self.drone_name)
      airsim.MultirotorState.kinematics_estimated
    return self.state



  def set_position(self, cmd):
    self.__time_start = time.time()
    self.__finished = False
    self.__cmd_timeout = cmd['timeout']
    self.__cancelCommand = False

    self.__cmd_thread = threading.Thread(target=Drone.__moveToPosition, args=(self, cmd))
    self.__cmd_thread.start()

  def __moveToPosition(self, cmd):
    if cmd["frame"] == "body":
      state = self.get_state()
      x_curr = state.kinematics_estimated.position.x_val
      y_curr = state.kinematics_estimated.position.y_val
      z_curr = state.kinematics_estimated.position.z_val

      (pitch, roll, yaw)  = airsim.to_eularian_angles(state.kinematics_estimated.orientation)
      
      x_cmd = cmd['x']*math.cos(yaw) - cmd['y']*math.sin(yaw) + x_curr
      y_cmd = cmd['x']*math.sin(yaw) + cmd['y']*math.cos(yaw) + y_curr
      z_cmd = cmd['z'] + z_curr
      yaw_cmd = cmd['yaw'] + math.degrees(yaw)
    
    elif cmd["frame"] == "world":
      x_cmd = cmd['x']
      y_cmd = cmd['y']
      z_cmd = cmd['z']
      yaw_cmd = cmd['yaw']
    
    else:
      print("DRONE " + self.drone_name + " POSITION CMD UNRECOGNIZED FRAME")

    yawmode = airsim.YawMode(False, yaw_cmd)

    with self.__client_lock:
      self.client.moveToPositionAsync(x_cmd, y_cmd, z_cmd, cmd['vel'], cmd['timeout'], yaw_mode=yawmode, vehicle_name=self.drone_name)
  
    with self.__threading_lock:
      desired = airsim.Vector3r(x_cmd, y_cmd, z_cmd)
      error = self.get_state().kinematics_estimated.position.distance_to(desired)
      
      while(time.time() - self.__time_start <= cmd['timeout'] and error > 0.05 and self.__shutdown == False and self.__cancelCommand == False):
        error = self.get_state().kinematics_estimated.position.distance_to(desired)
        time.sleep(0.2)

      '''
      print("DRONE " + self.drone_name + " FINISHED POSITION COMMAND")
      print("ERROR " + str(error))
      print("ELAPSED TIME " + str(time.time() - self.__time_start))
      print("MAX TIME " + str(cmd['timeout']) + "\n")
      print("CURRENT YAW " + str(yaw))
      print("YAW COMMAND " + str(yaw_cmd))
      '''

      self.__finished = True
      self.__publich_cmd = False


  def set_velocity(self, cmd):    
    self.__time_start = time.time()
    self.__finished = False
    self.__cmd_timeout = cmd['dur']
    self.__cancelCommand = False

    self.__cmd_thread = threading.Thread(target=Drone.__moveAtVelocity, args=(self, cmd))
    self.__cmd_thread.start()

  def __moveAtVelocity(self, cmd):
    if cmd["frame"] == "body": 
      while (time.time() - self.__time_start <= cmd['dur'] and self.__shutdown == False and self.__cancelCommand == False):
        (pitch, roll, yaw) = airsim.to_eularian_angles(self.get_state().kinematics_estimated.orientation)

        x = cmd['lx']*math.cos(yaw) - cmd['ly']*math.sin(yaw)
        y = cmd['lx']*math.sin(yaw) + cmd['ly']*math.cos(yaw)
        z = cmd['lz']
        yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(cmd['az']))

        with self.__client_lock:
          self.client.moveByVelocityAsync(x, y, z, 0.1, yaw_mode=yawmode, vehicle_name=self.drone_name)
        time.sleep(0.05)

    elif cmd["frame"] == "world":
      x = cmd['lx']
      y = cmd['ly']
      z = cmd['lz']
      dur = cmd['dur']
      yawmode = airsim.YawMode(is_rate=True, yaw_or_rate=cmd['az'])

      with self.__client_lock:
        self.client.moveByVelocityAsync(x, y, z, duration=dur, yaw_mode=yawmode, vehicle_name=self.drone_name)
      time.sleep(dur)  

    else:
      print("DRONE " + self.drone_name + " VEL cmd UNRECOGNIZED FRAME")
      self.hover()

    with self.__threading_lock:
      #print("DRONE " + self.drone_name + " FINISHED VELOCITY COMMAND")
      self.__finished = True
      self.__publich_cmd = False


  def hover(self, wait=False):
    self.__cancelCommand = True

    with self.__client_lock:
      if wait:
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
      else:
        self.client.hoverAsync(vehicle_name=self.drone_name)

    self.__finished = True


  def takeoff(self, wait=False):
    self.__cancelCommand = True

    if self.get_state().landed_state == airsim.LandedState.Flying:
      return

    with self.__client_lock:
      if wait:
        self.client.takeoffAsync(vehicle_name=self.drone_name).join()
      else:
        self.client.takeoffAsync(vehicle_name=self.drone_name)

    self.__finished = True        
      

  def land(self, wait=False):
    self.__cancelCommand = True

    if self.get_state().landed_state == airsim.LandedState.Landed:
      return
    
    with self.__client_lock:
      if wait:
        self.client.landAsync(vehicle_name=self.drone_name).join()
      else:
        self.client.landAsync(vehicle_name=self.drone_name)

    self.__finished = True

  def shutdown(self, shutdown=True):
    self.__shutdown = True
    self.__cancelCommand = True
    self.hover()

    with self.__client_lock:
      self.client.armDisarm(False, vehicle_name=self.drone_name)
      self.client.enableApiControl(is_enabled=False, vehicle_name=self.drone_name)

    if self.__cmd_thread != None:
      self.__cmd_thread.join()



  def wait_for_cmd(self):
    if self.__cmd_thread != None:
      self.__cmd_thread.join()