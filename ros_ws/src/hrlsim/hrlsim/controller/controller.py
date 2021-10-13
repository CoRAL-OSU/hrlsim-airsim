import abc
import numpy as np
from typing import Tuple, AnyStr

import hrlsim
import hrlsim.traj

class Controller(abc.ABC):
    def __init__(self, trajType: hrlsim.traj.Trajectory, maxThrust: float, mass: float) -> None:
        """
        Initilize controller

        Args:
            traj_generator (hrlsim.traj.Trajectory): Trajectory generator use to compute states
            maxThrust (float) Maximal possible thrust in body z direction
            mass (float) Mass of agent
        """
        self.traj_generator = trajType()
        pass

    @abc.abstractmethod
    def setGoals(self, waypoints: np.ndarray, ic:np.ndarray, fc:np.ndarray, avg_spd: float) -> None:
        """
        Sets the waypoints, initial conditions, and final conditions for the controller.

        Args:
            waypoints (np.ndarray): Waypoints to set
            ic (np.ndarray): Initial conditions [velocity, acceleration, jerk]
            fc (np.ndarray): Final conditions [velocity, acceleration, jerk]
            avg_spd (float): Average speed over trajectory
        """
        pass

    @abc.abstractmethod
    def computeControl(self, t:float, t0:float, dt: float, state: hrlsim.airsim.MultirotorState, drone_name: AnyStr) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the control for a given state

        Args:
            t0 (float): Initial time of trajectory
            state (MultirotorState): the state of the drone
            drone_name (string): Name of the drone

        Returns:
            Tuple[np.ndarray, np.ndarray, np.matrix]: Tuple representing
        """
        pass