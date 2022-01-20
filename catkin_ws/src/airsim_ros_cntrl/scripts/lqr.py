import numpy as np
import math
from typing import Any, List, Tuple
from airsim import Vector3r, Quaternionr, MultirotorState
import airsim
import rospy
import control

import minimum_snap


class LQR:
    """
    Class to compute the LQR controls.
    Uses minimum snap to compute trajectories
    """

    def __init__(self) -> None:
        """
        Constructs the intial LQR matrices
        """
        self.Q: np.ndarray = np.diag([100, 100, 100, 1, 1, 1, 1, 10, 10, 10])
        self.R: np.ndarray = np.diag([1, 1, 2e1, 1.0])

        self.K: np.ndarray = np.zeros((10, 10))

        self.mass = 1  # kg
        self.max_thrust = 4.1794 * 4  # N

        self.linearized_rotation = airsim.to_quaternion(0,0,0)
        self.first = True

        self.prev_gain_time = -999

    def set_costs(self, Q: List[int] = None, R: List[int] = None) -> None:
        """
        Sets the control matrices Q and R 

        Args:
            Q (List[int], optional): List of ints to set for Q. Must be length 10 Defaults to None.
            R (List[int], optional): List of ints to set for R. Must be length 4 Defaults to None.
        """
        if Q != None:
            assert len(Q) == 10, "Q must be a list of length 10"
            self.Q = np.diag(Q)

        if R != None:
            assert len(R) == 4, "R must be a list of length 4"
            self.R = np.diag(R)

    def set_goals(self, waypoints: np.ndarray, ic: np.ndarray, fc: np.ndarray, scaling_factor: float) -> None:
        """
        Sets the goals for the LQR controller.

        Args:
            waypoints (np.ndarray): Waypoints to set
        """
        tmp = np.copy(waypoints[0, :])

        waypoints[0, :] = waypoints[1, :]
        waypoints[1, :] = tmp
        waypoints[2, :] = -waypoints[2, :]

        tmp = np.copy(ic[:, 0])
        ic[:, 0] = ic[:, 1]
        ic[:, 1] = tmp
        ic[:, 2] = -ic[:, 2]

        tmp = np.copy(fc[:, 0])
        fc[:, 0] = fc[:, 1]
        fc[:, 1] = tmp
        fc[:, 2] = -fc[:, 2]

        self.traj_generator = minimum_snap.MinimumSnap(waypoints, ic, fc, scaling_factor)

    def updateGains(self, x: np.ndarray, omega: Vector3r, body_z_accel: float) -> None:
        """
        Updates the gains for the controller

        Args:
            x (np.ndarray): State, must be 10x1 state vector
            rpydot (Vector3r): Angular Vector 
            body_z_accel (int): Previous Acceleration
        """
        # c = (9.8-state.kinematics_estimated.linear_acceleration.z_val)/(cr*cp)
        # Compute thrust from command -> this is dangerous as there are unknown thrusts
        #   - start with open-loop from previous command
        #   - work to get motor thrust directly from AirSim
        #   - another option includes using nominal thrust, or 9.8/(cr*cp)

        pitch, roll, _ = airsim.to_eularian_angles(airsim.Quaternionr(x[4], x[5], x[6], x[3]))
        c = (9.8-body_z_accel)/(math.cos(roll)*math.cos(pitch))

        u = np.array([[omega[0], omega[1], omega[2], c]]).T

        for i in range(0, 4):
            if abs(u[i]) < 0.001:
                u[i] = 0.001

        A, B = self.computeLinearization(x, u)

        self.K, _, _ = control.lqr(A, B, self.Q, self.R)



    def computeControl(
        self, t0: float, state: MultirotorState, body_z_accel: int, drone_name
    ) -> Tuple[np.ndarray, np.ndarray, Any]:
        """
        Computes the control for a given state

        Args:
            t (float): the time
            state (MultirotorState): the state of the drone
            body_z_accel (int): previous acceleration

        Returns:
            Tuple[np.ndarray, np.ndarray, np.matrix]: Tuple representing
        """
        p = state.kinematics_estimated.position
        p = np.array([[p.x_val, p.y_val, p.z_val]]).T

        q = state.kinematics_estimated.orientation
        q = np.array([[q.w_val, q.x_val, q.y_val, q.z_val]]).T

        v = state.kinematics_estimated.linear_velocity
        v = np.array([[v.x_val, v.y_val, v.z_val]]).T

        x = np.concatenate((p,q,v) , 0)
        x = LQR.ned2xyz(x)

        rotation = (state.kinematics_estimated.orientation*self.linearized_rotation.inverse()).to_numpy_array()

        if 2*math.atan2(np.linalg.norm(rotation[0:3]), rotation[3]) > np.deg2rad(10) or self.first: 
            rospy.loginfo(str(drone_name) + " linearized: " + str(rospy.get_time() - self.prev_gain_time))

            self.prev_gain_time = rospy.get_time()

            self.updateGains(
                x, state.kinematics_estimated.angular_velocity.to_numpy_array(), body_z_accel
            )

            self.linearized_rotation = state.kinematics_estimated.orientation
            self.first = False

        x0, u0 = self.traj_generator.compute(rospy.get_time()-t0, x)

        u = np.zeros((4, 1))

        # Compute the optimal control step
        u = u0 - np.matmul(self.K, (x - x0))

        tmp = u[0, 0]
        u[0, 0] = u[1, 0]
        u[1, 0] = tmp
        u[2, 0] = u[2, 0]

        u[3, 0] = abs(u[3, 0]) * self.mass / self.max_thrust

        x0 = LQR.xyz2ned(x0)
        return x0, u

    def thrust2world(self, state: MultirotorState, throttle: np.matrix) -> np.ndarray:
        """
        Converts thrust to acceleration vector

        Args:
            state (MultirotorState): State of the drone
            throttle (np.matrix): matrix representing the throttle

        Returns:
            np.ndarray: Acceleration vector
        """
        pitch, roll, yaw = airsim.to_eularian_angles(state.kinematics_estimated.orientation)

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        accelF = (throttle) * self.max_thrust / self.mass
        accel = np.zeros((3))

        accel[0] = (-cr * sp * cy - sr * sy) * accelF
        accel[1] = (-cr * sp * sy + sr * cy) * accelF
        accel[2] = 9.8 - (cr * cp) * accelF

        return accel


    @staticmethod
    def xyz2ned(x: np.ndarray) -> np.ndarray:
        """
        Convert X,Y,Z to North, East, Down

        Args:
            x (np.ndarray): North, East, Down

        Returns:
            np.ndarray: X, Y, Z
        """

        assert np.shape(x) == (10,1), "The state must be a 10x1 vector"

        xnew = np.zeros((10,1))
        xnew[0] = x[1]
        xnew[1] = x[0]
        xnew[2] = -x[2]

        q1 = Quaternionr(x[4], x[5], x[6], x[3])
        q_rot = Quaternionr(math.sqrt(2)/2, -math.sqrt(2)/2, 0, 0)

        qnew = (q1.rotate(q_rot)).to_numpy_array()
        xnew[4:7] = qnew[0:3]
        xnew[3] = qnew[3]
        
        xnew[7] = x[8]
        xnew[8] = x[7]
        xnew[9] = -x[9]

        return xnew


    @staticmethod
    def ned2xyz(x: np.ndarray) -> np.ndarray:
        """
        Convert North, East, Down to X, Y, Z

        Args:
            x (np.ndarray): North, East, Down

        Returns:
            np.ndarray: X, Y, Z
        """
        assert np.shape(x) == (10, 1), "The state must be a 10x1 vector"
        xnew = np.zeros((10, 1))

        xnew[0] = x[1]
        xnew[1] = x[0]
        xnew[2] = -x[2]

        q1 = Quaternionr(x[4], x[5], x[6], x[3])
        q_rot = Quaternionr(math.sqrt(2) / 2, math.sqrt(2) / 2, 0, 0)

        qnew = (q1.rotate(q_rot)).to_numpy_array()

        xnew[3] = qnew[3]
        xnew[4:7] = qnew[0:3]

        xnew[7] = x[8]
        xnew[8] = x[7]
        xnew[9] = -x[9]

        return xnew

    @staticmethod
    def get_state(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Gets the state from a array

        Args:
            x (np.ndarray): the state

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: Tuple representing, Position, Orientation, Velocity
        """
        assert np.shape(x) == (10, 1), "The state must be a 10x1 vector"
        return x[0:3], x[3:7], x[7:10]

    @staticmethod
    def get_command(u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Gets the command from an array

        Args:
            u (np.ndarray): The command

        Returns:
            Tuple[np.ndarray, np.ndarray]: Tuple representing body rates
        """
        assert np.shape(u) == (4, 1), "The command must be a 4x1 vector"
        return u[0:3], u[3][0]


    def computeLinearization(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Updates A maxtrix from state and command lists

        Args:
            x (np.ndarray): The state list
            u (np.ndarray): The command list

        Returns:
            np.ndarray: A matrix
            np.ndarray: B matrix
        """
        _, q, _ = LQR.get_state(x)
        omega, c = LQR.get_command(u)


        ### COMPUTE PARTIAL DERIVATIVES ###
        #

        # Compute qdotq
        Q = np.array([[0.0, -omega[0,0], -omega[1,0], -omega[2,0]],
                      [omega[0,0], 0.0, omega[2,0], -omega[1,0]],
                      [omega[1,0], -omega[2,0], 0.0, omega[0,0]],
                      [omega[2,0], omega[1,0], -omega[0,0], 0.0]])

        qnorm = np.linalg.norm(q)
        qdotq =  0.5 * np.matmul(
            Q, (np.identity(4) - (math.pow(qnorm, -2) * q * q.T)) / qnorm
        )



        # Compute vdotq
        Q = np.array([[ q[2,0], q[3,0], q[0,0], q[1,0]],
                      [-q[1,0], -q[0,0], q[3,0], q[2,0]],
                      [ q[0,0], -q[1,0], -q[2,0], q[3,0]]])

        qnorm = np.linalg.norm(q)
        vdotq = (2.0*c*np.matmul(Q, (np.identity(4) - (math.pow(qnorm, -2) * q * q.T)) / qnorm))



        # Compute qdotomega
        qdotomega = 0.5*np.array([[-q[1,0], -q[2,0], -q[3,0]],
                            [ q[0,0], -q[3,0], -q[2,0]],
                            [ q[3,0],  q[0,0],  q[1,0]],
                            [-q[2,0],  q[1,0],  q[0,0]]])


        # Compute vdotc
        vdotc = np.array([[q[0,0] * q[2,0] + q[1,0] * q[3,0]],
                          [q[2,0] * q[3,0] - q[0,0] * q[1,0]],
                          [q[0,0] * q[0,0] - q[1,0] * q[1,0] - q[2,0] * q[2,0] + q[3,0] * q[3,0]]])


        #### USE PARTIALS TO COMPUTE LINEARIZATION ###
        #

        # 10x10 block matrix
        A = np.block(
            [
                [np.zeros((3, 3)), np.zeros((3, 4)), np.identity(3)],
                [np.zeros((4, 3)), qdotq, np.zeros((4, 3))],
                [np.zeros((3, 3)), vdotq, np.zeros((3, 3))],
            ]
        )

        # 10x4 block matrix
        B = np.block(
            [
                [np.zeros((3, 3)), np.zeros((3, 1))],
                [qdotomega, np.zeros((4, 1))],
                [np.zeros((3, 3)), vdotc],
            ]
        )

        return A, B