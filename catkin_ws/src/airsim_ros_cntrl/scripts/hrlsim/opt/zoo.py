import numpy as np
from typing import Tuple, AnyStr

from . import optimization

class ZOO(optimization.Optimization):
    def __init__(self, eta: Tuple, delta: float) -> None:
        """
        Initilize optimizer

        Args:
        eta (float): Default step size
        """
        super().__init__(eta)
        self.delta = delta
        self.V0 = None


    def computeStep(self, V: float, X: np.ndarray, eta = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the control for a given state

        Args:
            V (float): Cost of current iteration
            X (np.ndarray): Starting point for optimization
            eta (float): Step size (None -> Use self.eta)

        Returns:
            Tuple[np.ndarray, np.ndarray, np.matrix]: Tuple representing
        """

        u = np.random.randn(*X.shape)
        if self.V0 == None:
            self.V0 = V
            return (X + u*self.delta, u*self.delta)
            #gtilde = u/self.delta*V
        
        else:
            gtilde = u/self.delta * (V - self.V0)

        self.V0 = V

        if eta == None:
            eta = self.eta

        step = eta*gtilde
        Xtilde = (X - step) +u*self.delta

        return (Xtilde, step)
