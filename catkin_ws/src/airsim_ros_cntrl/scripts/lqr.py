#! /usr/bin/python2

import time
import multiprocessing as mp
import numpy as np
import math
import sys

import airsim
import slycot
import control

import minimum_snap


class LQR:
    def __init__(self):
        self.Q = np.diag([100, 100, 100, 1, 1, 1, 1, 10, 10, 10])
        self.R = np.diag([1e1, 1e1, 2e1, 2])

        self.A = np.zeros((10, 10))
        self.B = np.zeros((10, 4))
        self.K = np.zeros((10, 10))

        self.mass = 1  # kg
        self.max_thrust = 4.1794 * 4  # N

        self.update_gain_period = 0.2  # seconds
        self.prev_gain_time = time.time()

    def set_costs(self, Q=None, R=None):
        if Q != None:
            assert len(Q) == 10, "Q must be a list of length 10"
            self.Q = np.diag(Q)

        if R != None:
            assert len(R) == 4, "R must be a list of length 4"
            self.R = np.diag(R)

    def set_goals(self, waypoints):

        tmp = np.copy(waypoints[0, :])

        waypoints[0, :] = waypoints[1, :]
        waypoints[1, :] = tmp
        waypoints[2, :] = -waypoints[2, :]

        self.traj_generator = minimum_snap.MinimumSnap(waypoints)

    def updateGains(self, x, rpydot, prev_accel_cmd):
        roll, pitch, _ = LQR.quat2rpy(x[3:7])

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        tp = math.tan(pitch)

        R = np.matrix([[1, sr * tp, cr * tp], [0, cr, -sr], [0, sr / cp, cr / cp]])

        rpydot = np.array([[rpydot.y_val, -rpydot.x_val, rpydot.z_val]]).T
        omega = R.I * rpydot
        omega = [omega[0, 0], omega[1, 0], omega[2, 0]]

        # c = (9.8-state.kinematics_estimated.linear_acceleration.z_val)/(cr*cp)
        # Compute thrust from command -> this is dangerous as there are unknown thrusts
        #   - start with open-loop from previous command
        #   - work to get motor thrust directly from AirSim
        #   - another option includes using nominal thrust, or 9.8/(cr*cp)

        c = (9.8 - prev_accel_cmd) / (cr * cp)

        u = LQR.set_command(omega, c)

        self.__updateGains(x, u)
        self.prev_gain_time = time.time()

    def __updateGains(self, x, u):
        assert np.shape(x) == (10, 1), "The state must be a 10x1 state vector"
        assert np.shape(u) == (4, 1), "The goal command must be a 4x1 control vector"

        for i in range(0, 4):
            if abs(u[i]) < 0.001:
                u[i] = 0.001

        self.A = self.__updateA(x, u)
        self.B = self.__updateB(x)

        """
        Co = control.ctrb(self.A,self.B)

        try:
            assert np.linalg.matrix_rank(self.A) <= np.linalg.matrix_rank(Co), "System is not controllable"
        
        except AssertionError:
            print("System is not controllable")
            exit(1)     
        """

        self.K, _, _ = control.lqr(self.A, self.B, self.Q, self.R)

        return self.K

    def computeControl(self, t, state, prev_accel_cmd):
        p = state.kinematics_estimated.position
        p = [p.x_val, p.y_val, p.z_val]

        q = state.kinematics_estimated.orientation
        q = [q.w_val, q.x_val, q.y_val, q.z_val]

        v = state.kinematics_estimated.linear_velocity
        v = [v.x_val, v.y_val, v.z_val]

        roll, pitch, _ = LQR.quat2rpy(q)

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        tp = math.tan(pitch)

        R = np.matrix([[1, sr * tp, cr * tp], [0, cr, -sr], [0, sr / cp, cr / cp]])

        x = LQR.set_state(p, q, v)
        x = LQR.ned2xyz(x)

        x0, u0 = self.traj_generator.compute(t, x)

        if time.time() - self.prev_gain_time > self.update_gain_period:
            self.updateGains(
                x, state.kinematics_estimated.angular_velocity, prev_accel_cmd
            )
            self.prev_gain_time = time.time()

        u = np.zeros((4, 1))

        # Compute the optimal control step
        u = u0 - np.matmul(self.K, (x - x0))

        tmp = u[0, 0]
        u[0, 0] = u[1, 0]
        u[1, 0] = tmp
        u[2, 0] = -u[2, 0]

        u[3, 0] = abs(u[3, 0]) * self.mass / self.max_thrust

        u[0:3] = R * u[0:3]

        u[1, 0] = -u[1, 0]
        u[2, 0] = -u[2, 0]

        tmp = x0[0, 0]
        x0[0, 0] = x0[1, 0]
        x0[1, 0] = tmp
        x0[2, 0] = -x0[2, 0]

        tmp = x0[4, 0]
        x0[4, 0] = x0[5, 0]
        x0[5, 0] = tmp
        x0[6, 0] = -x0[6, 0]

        tmp = x0[7, 0]
        x0[7, 0] = x0[8, 0]
        x0[8, 0] = tmp
        x0[9, 0] = -x0[9, 0]

        tmp = u0[0, 0]
        u0[0, 0] = u0[1, 0]
        u0[1, 0] = tmp
        u0[2, 0] = -u0[2, 0]

        return x0, u0, u

    def thrust2world(self, state, throttle):
        q = state.kinematics_estimated.orientation
        q = [q.w_val, q.x_val, q.y_val, q.z_val]

        roll, pitch, yaw = LQR.quat2rpy(q)

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
    def quat2rpy(q):
        roll = math.atan2(
            2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1]) * q[1] + q[2] * q[2]
        )
        pitch = math.asin(2 * (q[0] * q[2] - q[3] * q[1]))
        yaw = math.atan2(
            2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        )
        return roll, pitch, yaw

    @staticmethod
    def rpy2quat(roll, pitch, yaw):
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
        return q

    @staticmethod
    def ned2xyz(x):
        assert np.shape(x) == (10, 1), "The state must be a 10x1 vector"
        # tp = np.matrix([[0,1,0],[1,0,0],[0,0,-1]])
        # tq = np.block([[1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,-1]])

        # T = np.block([[ tp, np.zeros((3,4)), np.zeros((3,3)) ],
        #              [ np.zeros((4,3)), tq, np.zeros((4,3)) ],
        #              [ np.zeros((3,3)), np.zeros((3,4)), tp ]])

        xnew = np.zeros((10, 1))

        xnew[0] = x[1]
        xnew[1] = x[0]
        xnew[2] = -x[2]

        # Apply quaternion rotation to  translate. Don't swap axis
        # R =[1 0 0 ; 0 -1 0; 0 0 -1]
        q1 = airsim.Quaternionr(x[4], x[5], x[6], x[3])
        q_rot = airsim.Quaternionr(math.sqrt(2) / 2, math.sqrt(2) / 2, 0, 0)

        qnew = q1.rotate(q_rot)
        qnew = qnew.to_numpy_array()

        xnew[3] = qnew[3]
        xnew[4] = qnew[0]
        xnew[5] = qnew[1]
        xnew[6] = qnew[2]

        xnew[7] = x[8]
        xnew[8] = x[7]
        xnew[9] = -x[9]

        x = xnew  # np.matmul(T,x)
        return x

    @staticmethod
    def get_state(x):
        assert np.shape(x) == (10, 1), "The state must be a 10x1 vector"
        return x[0:3], x[3:7], x[7:10]

    @staticmethod
    def get_command(u):
        assert np.shape(u) == (4, 1), "The command must be a 4x1 vector"
        return u[0:3], u[3][0]

    @staticmethod
    def set_state(p, q, v):
        assert len(p) == 3, "The position must be a 3x1 list"
        assert len(q) == 4, "The quaternion must be a 4x1 list"
        assert len(v) == 3, "The velocity must be a 3x1 list"

        p = np.array([p], dtype="float").T
        q = np.array([q], dtype="float").T
        v = np.array([v], dtype="float").T

        x = np.block([[p], [q], [v]])
        return x

    @staticmethod
    def set_command(omega, c):
        assert len(omega) == 3, "The body rates must be a 3x1 list"

        omega = np.array([omega], dtype="float").T

        u = np.block([[omega], [c]])
        return u

    def __updateA(self, x, u):
        p, q, v = self.get_state(x)
        omega, c = self.get_command(u)

        # 10x10 block matrix
        A = np.block(
            [
                [np.zeros((3, 3)), np.zeros((3, 4)), np.identity(3)],
                [np.zeros((4, 3)), LQR.__qdotq(omega, q), np.zeros((4, 3))],
                [np.zeros((3, 3)), LQR.__vdotq(c, q), np.zeros((3, 3))],
            ]
        )
        return A

    def __updateB(self, x):
        p, q, v = LQR.get_state(x)

        # 10x4 block matrix
        B = np.block(
            [
                [np.zeros((3, 3)), np.zeros((3, 1))],
                [LQR.__qdotomega(q), np.zeros((4, 1))],
                [np.zeros((3, 3)), LQR.__vdotc(q)],
            ]
        )
        return B

    @staticmethod
    def __qdotq(omega, q):
        y = np.block(
            [
                [0.0, -omega[0], -omega[1], -omega[2]],
                [omega[0], 0.0, omega[2], -omega[1]],
                [omega[1], -omega[2], 0.0, omega[0]],
                [omega[2], omega[1], -omega[0], 0.0],
            ]
        )

        qnorm = np.linalg.norm(q)
        return 0.5 * np.matmul(
            y, (np.identity(4) - (math.pow(qnorm, -2) * q * q.T)) / qnorm
        )

    @staticmethod
    def __vdotq(c, q):
        y = np.block(
            [
                [q[2], q[3], q[0], q[1]],
                [-q[1], -q[0], q[3], q[2]],
                [q[0], -q[1], -q[2], q[3]],
            ]
        )

        qnorm = np.linalg.norm(q)

        return (
            2.0
            * c
            * np.matmul(y, (np.identity(4) - (math.pow(qnorm, -2) * q * q.T)) / qnorm)
        )

    @staticmethod
    def __qdotomega(q):
        y = np.block(
            [
                [-q[1], -q[2], -q[3]],
                [q[0], -q[3], -q[2]],
                [q[3], q[0], q[1]],
                [-q[2], q[1], q[0]],
            ]
        )
        return 0.5 * y

    @staticmethod
    def __vdotc(q):
        y = np.block(
            [
                [q[0] * q[2] + q[1] * q[3]],
                [q[2] * q[3] - q[0] * q[1]],
                [q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]],
            ]
        )
        return y
