#! /usr/bin/python3

from airsim.client import MultirotorClient
from airsim.types import DrivetrainType, Vector3r, YawMode

import airsim, rospy, pymap3d



class Camera:

    def __init__(self, client: MultirotorClient, drone_name: str):
        self.client = client
        self.drone_name = drone_name

        self.client.enableApiControl(True, drone_name)

    def takeoff(self, wait=False):
        future = self.client.takeoffAsync(vehicle_name=self.drone_name)
        
        if wait:
            future.join()

    def land(self, wait=False):
        future = self.client.landAsync(vehicle_name=self.drone_name)

        if wait:
            future.join()

    def moveToLocation(self, location: Vector3r, velocity: float, yaw: float, wait=False):
        future = self.client.moveToPositionAsync(location.x_val, location.y_val, location.z_val, velocity, yaw_mode=YawMode(False, yaw), vehicle_name=self.drone_name)
        
        if wait:
            future.join()

    def moveOnPath(self, path: list([Vector3r]), velocity: float, yaw: float, wait=False):
        future = self.client.moveOnPathAsync(path, velocity, yaw_mode=YawMode(False, yaw), vehicle_name=self.drone_name)

        if wait:
            future.join()

    def getLocation(self) -> Vector3r:
        location = self.client.getGpsData(vehicle_name=self.drone_name)
        lat = location.gnss.geo_point.latitude
        lon = location.gnss.geo_point.longitude
        alt = location.gnss.geo_point.altitude


        home = self.client.getHomeGeoPoint()
        lat0 = home.latitude
        lon0 = home.longitude
        alt0 = home.altitude

        (n,e,d) = pymap3d.geodetic2ned(lat, lon, alt, lat0, lon0, alt0)

        return Vector3r(n,e,d)


if __name__ == "__main__":

    client = MultirotorClient(ip="10.0.0.3")
    client.confirmConnection()

    camera_drone = Camera(client, "Camera")

    print("Taking off")
    camera_drone.takeoff()
    
    print("Getting location")
    location = camera_drone.getLocation()

    print("Location: " + str(location))


    print("Moving")
    camera_drone.moveToLocation(Vector3r(0,0,-5), 2, -10, wait=True)


    print("Finished")

