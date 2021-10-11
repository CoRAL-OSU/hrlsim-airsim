import abc
import numpy as np
from typing import Tuple

class Optimization(abc.ABC):
    def __init__(self, eta:float) -> None:
        """
        Initilize optimizer

        Args:
        eta (float): Default step size
        """
        self.eta = eta
        pass


    @abc.abstractmethod
    def computeStep(self, V: float, X: np.ndarray, eta = None) -> Tuple[float, float]:
        """
        Computes the control for a given state

        Args:
            V (float): Cost of current iteration
            X (np.ndarray): Starting point for optimization
            eta (float): Step size (None -> Use self.eta)

        Returns:
            Tuple[float, float]
            (New value, step size)
        """
        pass