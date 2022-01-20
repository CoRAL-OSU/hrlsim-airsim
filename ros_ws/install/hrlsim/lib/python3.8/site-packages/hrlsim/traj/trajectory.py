import abc
import numpy as np
from typing import Tuple



class Trajectory(abc.ABC):
    def __init__(self):
        """
        Initialize the generator
        """
        pass

    @abc.abstractmethod
    def generate(self, waypoints: np.ndarray, ic: np.ndarray, fc: np.ndarray, avg_spd: float) -> None:
        """
        Generate a trajectory

        Args:
            waypoints (np.ndarray): Waypoints to set
            ic (np.ndarray): Initial conditions [velocity, acceleration, jerk]
            fc (np.ndarray): Final conditions [velocity, acceleration, jerk]
            avg_spd (float): Average speed over trajectory
        """
        pass

    @abc.abstractmethod
    def compute(self, t: float, state: np.ndarray, compute_control=True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the next trajectory from time and state

        Args:
            t (float): The current time
            state (np.ndarray): the state list
            compute_control (bool): if true, return control as second value. else, return linear acceleration

        Returns:
            Tuple[np.ndarray, np.ndarray]: Tuple of lists
        """
        pass


class DesiredState:
    """
    Class to represent the desired final state
    """

    def __init__(self) -> None:
        """
        Constructs a desired state
        """
        self.pos = np.zeros((3, 1))
        self.vel = np.zeros((3, 1))
        self.acc = np.zeros((3, 1))
        self.yaw = 0
        self.yawdot = 0