#! /usr/bin/python3

import numpy as np
import rclpy
import math
from typing import Tuple

import hrlsim.airsim
from . import trajectory

class MinimumSnap(trajectory.Trajectory):
    """
    Class to represent a minimum snap object.
    Used to compute the desired trajectory.
    """
    def generate(self, waypoints: np.ndarray, ic: np.ndarray, fc: np.ndarray, avg_spd: float) -> None:
        """
        Generate a trajectory with minimum snap

        Args:
            waypoints (np.ndarray): Waypoints to set
            ic (np.ndarray): Initial conditions [velocity, acceleration, jerk]
            fc (np.ndarray): Final conditions [velocity, acceleration, jerk]
            avg_spd (float): Average speed over trajectory
        """
        self.t = rclpy.get_clock().now()
        self.state = np.zeros((10, 1))

        d = waypoints[:, 1:] - waypoints[:, 0:-1]
        
        self.d0 = (
            np.sqrt(d[0,:] * d[0,:] + d[1,:] * d[1,:] + d[2,:] * d[2,:]) / avg_spd
        )

        # Bound the time so we're not too aggresive with small changes in position. 
        #! Verify this with tracking
        
        if len(self.d0) > 0:
            self.d0 = np.maximum(self.d0, 2*np.ones(self.d0.size))

        self.traj_time = np.append(0, np.cumsum(self.d0))
        self.waypoints0 = np.copy(waypoints)

        N = np.size(waypoints, 1) - 1

        self.p_c = np.zeros((7, 8))

        self.p_c[0,:] = np.ones((1, 8))
        self.p_c[1,:] = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        self.p_c[2,:] = np.array([0, 0, 2, 6, 12, 20, 30, 42])
        self.p_c[3,:] = np.array([0, 0, 0, 6, 24, 60, 120, 210])
        self.p_c[4,:] = np.array([0, 0, 0, 0, 24, 120, 360, 840])
        self.p_c[5,:] = np.array([0, 0, 0, 0, 0, 120, 720, 2520])
        self.p_c[6,:] = np.array([0, 0, 0, 0, 0, 0, 720, 5040])

        head_c = np.diag(self.p_c)
        head_c = np.diag(head_c)

        head_c = np.append(head_c, np.zeros((7, 1)), 1)

        A = np.zeros((8*N, 8*N))
        b = np.zeros((8*N, 3))

        for i in range(1, N + 1):

            A[8*(i-1), np.arange(0,8) + 8*(i-1)] = head_c[0,:]
            b[8*(i-1),:] = waypoints[:, i-1].T

            A[8*(i-1)+1, np.arange(0,8) + 8*(i-1)] = self.p_c[0,:]
            b[8*(i-1)+1,:] = waypoints[:,i].T

            if i < N:
                A[8*(i-1)+2, np.arange(0,16) + 8*(i-1)] = np.append(self.p_c[1,:], -head_c[1,:])
                A[8*(i-1)+3, np.arange(0,16) + 8*(i-1)] = np.append(self.p_c[2,:], -head_c[2,:])
                A[8*(i-1)+4, np.arange(0,16) + 8*(i-1)] = np.append(self.p_c[3,:], -head_c[3,:])
                A[8*(i-1)+5, np.arange(0,16) + 8*(i-1)] = np.append(self.p_c[4,:], -head_c[4,:])
                A[8*(i-1)+6, np.arange(0,16) + 8*(i-1)] = np.append(self.p_c[5,:], -head_c[5,:])
                A[8*(i-1)+7, np.arange(0,16) + 8*(i-1)] = np.append(self.p_c[6,:], -head_c[6,:])

        if N > 0:
            b[8*N-6] = ic[0]*self.d0[0]
            b[8*N-5] = ic[1]*self.d0[0]
            b[8*N-4] = ic[2]*self.d0[0]
            b[8*N-3] = fc[0]*self.d0[-1]
            b[8*N-2] = fc[1]*self.d0[-1]
            b[8*N-1] = fc[2]*self.d0[-1]

            A[8*N-6, np.arange(0,8)] = head_c[1,:]
            A[8*N-5, np.arange(0,8)] = head_c[2,:]
            A[8*N-4, np.arange(0,8)] = head_c[3,:]
            A[8*N-3, np.arange(0,8) + 8*(N-1)] = self.p_c[1,:]
            A[8*N-2, np.arange(0,8) + 8*(N-1)] = self.p_c[2,:]
            A[8*N-1, np.arange(0,8) + 8*(N-1)] = self.p_c[3,:]

            x1 = np.matmul(np.linalg.inv(A), b[:,0])
            x2 = np.matmul(np.linalg.inv(A), b[:,1])
            x3 = np.matmul(np.linalg.inv(A), b[:,2])

            self.alpha = np.zeros((8,N,3))

            self.alpha[:,:,0] = np.reshape(x1, (8,N), "F")
            self.alpha[:,:,1] = np.reshape(x2, (8,N), "F")
            self.alpha[:,:,2] = np.reshape(x3, (8,N), "F")



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
        
        if t > self.traj_time[-1]:
            t = self.traj_time[-1]

        t_index = 0
        for i in range(0, len(self.traj_time)):
            if self.traj_time[i] >= t:
                t_index = i
                break

        if t_index > 0:
            t = t - self.traj_time[t_index - 1]


        desired_state = hrlsim.traj.DesiredState()

        if self.waypoints0.shape[1] > 1:
            scale = t / self.d0[t_index - 1]

            f_p = np.squeeze(self.alpha[:,t_index-1,:]).T * np.reshape(np.repeat(self.p_c[0,:],3), (3,8), "F")
            f_p = np.flip(f_p,1)
            desired_state.pos = np.array([[np.polyval(f_p[0,:], scale)],
                                          [np.polyval(f_p[1,:], scale)],
                                          [np.polyval(f_p[2,:], scale)]])

            f_v = np.squeeze(self.alpha[:,t_index-1,:]).T * np.reshape(np.repeat(self.p_c[1,:],3), (3,8), "F")
            f_v = np.flip(f_v[:,np.arange(1,8)], 1)
            desired_state.vel = np.array([[np.polyval(f_v[0, :], scale)],
                                          [np.polyval(f_v[1, :], scale)],
                                          [np.polyval(f_v[2, :], scale)]])
            desired_state.vel /= self.d0[t_index-1]

            f_a = np.squeeze(self.alpha[:,t_index-1,:]).T * np.reshape(np.repeat(self.p_c[2,:],3), (3,8), "F")
            f_a = np.flip(f_a[:,np.arange(2,8)], 1)
            desired_state.acc = np.array([[np.polyval(f_a[0, :], scale)],
                                          [np.polyval(f_a[1, :], scale)],
                                          [np.polyval(f_a[2, :], scale)]])
            desired_state.acc /= self.d0[t_index-1]**2

        else:
            desired_state.pos = self.waypoints0[:,0]

        dx = self.waypoints0[0,-1] - state[0,0]
        dy = self.waypoints0[1,-1] - state[1,0]

        _, _, cyaw = hrlsim.airsim.to_eularian_angles(hrlsim.airsim.Quaternionr(state[4], state[5], state[6], state[3]))

        if dx ** 2 + dy ** 2 < 1:
            desired_state.yaw = cyaw  # 0
        else:
            desired_state.yaw = cyaw  # -math.atan2(dy,dx)

        desired_state.yawdot = 0

        x0 = np.zeros((10, 1))
        x0[0:3]  = np.array([desired_state.pos]).T
        x0[7:10] = desired_state.vel

        yaw = desired_state.yaw

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        roll  = (1/9.8) * (desired_state.acc[0,0]*sy - desired_state.acc[1,0]*cy)
        pitch = (1/9.8) * (desired_state.acc[0,0]*cy + desired_state.acc[1,0]*sy)

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        q = hrlsim.airsim.to_quaternion(pitch, roll, yaw).to_numpy_array()
        x0[3,0]   = q[3]
        x0[4:7,0] = q[0:3]


        u0 = np.zeros((4, 1))

        omega_r = cr * cp * desired_state.yawdot

        if not compute_control:
            return x0, desired_state.acc

        u0[2] = omega_r

        R_BA = np.array([[ cr*cp - sr*sy*sp, -cr*sy,   cy*sp + cp*sr*sy],
                         [ cp*sy + cy*sp*sr,  cr*cy,   sy*sp - cy*cp*sr],
                         [-cr*sp,             sr,      cr*cp]])

        c = np.matmul(np.linalg.inv(R_BA), np.array([[0, 0, desired_state.acc[2,0]+9.8]]).T)
        u0[3] = c[2]

        return x0, u0