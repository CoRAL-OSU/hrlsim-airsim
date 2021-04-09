#! /usr/bin/python2

import time
import numpy as np
import math
from typing import Tuple

import matplotlib.pyplot as plt

import lqr


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


class MinimumSnap:
    """
    Class to represent a minimum snap object.
    Used to compute the desired trajectory.
    """
    def __init__(self, waypoints: np.ndarray, initial_conditions: np.ndarray, final_conditions: np.ndarray, scaling_factor: np.ndarray) -> None:
        """
        Constructs a minimum snap object from the desired waypoints

        Args:
            waypoints (np.ndarray): Desired Waypoints
        """
        self.t = time.time()
        self.state = np.zeros((10, 1))

        d = waypoints[:, 1:] - waypoints[:, 0:-1]

        #self.d0 = np.array([1/scaling_factor])
        self.d0 = np.empty((3,0))

        for i in range(0, d.shape[1]):
            segment_time = np.array([[d[0,i]/scaling_factor[0], d[1,i]/scaling_factor[1], d[2,i]/scaling_factor[2]]]).T
            self.d0 = np.append(self.d0, segment_time, 1)

        #self.d0 = (
        #    np.sqrt(d[0, :] * d[0, :] + d[1, :] * d[1, :] + d[2, :] * d[2, :]) / scaling_factor
        #)

        self.traj_time = np.append(0, np.cumsum(self.d0))
        self.waypoints0 = np.copy(waypoints)

        N = np.size(waypoints, 1) - 1

        self.p_c = np.zeros((7, 8))

        self.p_c[0, :] = np.ones((1, 8))
        self.p_c[1, :] = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        self.p_c[2, :] = np.array([0, 0, 2, 6, 12, 20, 30, 42])
        self.p_c[3, :] = np.array([0, 0, 0, 6, 24, 60, 120, 210])
        self.p_c[4, :] = np.array([0, 0, 0, 0, 24, 120, 360, 840])
        self.p_c[5, :] = np.array([0, 0, 0, 0, 0, 120, 720, 2520])
        self.p_c[6, :] = np.array([0, 0, 0, 0, 0, 0, 720, 5040])

        head_c = np.diag(self.p_c)
        head_c = np.diag(head_c)

        head_c = np.append(head_c, np.zeros((7, 1)), 1)

        A = np.zeros((8 * N, 8 * N))
        b = np.zeros((8 * N, 3))

        for i in range(1, N + 1):

            A[(i - 1) * 8, np.arange(0, 8) + (i - 1) * 8] = head_c[0, :]
            b[(i - 1) * 8, :] = waypoints[:, i - 1].T

            A[(i - 1) * 8 + 1, np.arange(0, 8) + (i - 1) * 8] = self.p_c[0, :]
            b[(i - 1) * 8 + 1, :] = waypoints[:, i].T

            if i < N:
                A[(i - 1) * 8 + 2, np.arange(0, 16) + (i - 1) * 8] = np.append(
                    self.p_c[1, :], -head_c[1, :]
                )
                A[(i - 1) * 8 + 3, np.arange(0, 16) + (i - 1) * 8] = np.append(
                    self.p_c[2, :], -head_c[2, :]
                )
                A[(i - 1) * 8 + 4, np.arange(0, 16) + (i - 1) * 8] = np.append(
                    self.p_c[3, :], -head_c[3, :]
                )
                A[(i - 1) * 8 + 5, np.arange(0, 16) + (i - 1) * 8] = np.append(
                    self.p_c[4, :], -head_c[4, :]
                )
                A[(i - 1) * 8 + 6, np.arange(0, 16) + (i - 1) * 8] = np.append(
                    self.p_c[5, :], -head_c[5, :]
                )
                A[(i - 1) * 8 + 7, np.arange(0, 16) + (i - 1) * 8] = np.append(
                    self.p_c[6, :], -head_c[6, :]
                )

        b[8*N-6, :] = initial_conditions[0,:]*self.d0[0]
        b[8*N-5, :] = initial_conditions[1,:]*self.d0[0]
        b[8*N-4, :] = initial_conditions[2,:]*self.d0[0]
        b[8*N-3, :] = final_conditions[0,:]*self.d0[-1]
        b[8*N-2, :] = final_conditions[1,:]*self.d0[-1]
        b[8*N-1, :] = final_conditions[2,:]*self.d0[-1]

        A[8 * N - 6, np.arange(0, 8)] = head_c[1, :]
        A[8 * N - 5, np.arange(0, 8)] = head_c[2, :]
        A[8 * N - 4, np.arange(0, 8)] = head_c[3, :]
        A[8 * N - 3, np.arange(0, 8) + 8 * (N - 1)] = self.p_c[1, :]
        A[8 * N - 2, np.arange(0, 8) + 8 * (N - 1)] = self.p_c[2, :]
        A[8 * N - 1, np.arange(0, 8) + 8 * (N - 1)] = self.p_c[3, :]

        x1 = np.matmul(np.linalg.inv(A), b[:, 0])
        x2 = np.matmul(np.linalg.inv(A), b[:, 1])
        x3 = np.matmul(np.linalg.inv(A), b[:, 2])

        self.alpha = np.zeros((8, N, 3))

        self.alpha[:, :, 0] = np.reshape(x1, (8, N), "F")
        self.alpha[:, :, 1] = np.reshape(x2, (8, N), "F")
        self.alpha[:, :, 2] = np.reshape(x3, (8, N), "F")



    def compute(self, t: float, state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the next trajectory from time and state

        Args:
            t (float): The current time
            state (np.ndarray): the state list

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

        desired_state = DesiredState()

        #if t == 0:
        #    desired_state.pos = self.waypoints0[:, 0]
        #    desired_state.vel = state[7:9, 0]  # np.zeros((3,1))
        #    desired_state.acc = np.zeros((3, 1))
        #else:
        scale = t / self.d0[t_index - 1]

        f_p = np.squeeze(self.alpha[:, t_index - 1, :]).T * np.reshape(
            np.repeat(self.p_c[0, :], 3), (3, 8), "F"
        )
        f_p = np.flip(f_p, 1)
        desired_state.pos = np.array(
            [
                [np.polyval(f_p[0, :], scale)],
                [np.polyval(f_p[1, :], scale)],
                [np.polyval(f_p[2, :], scale)],
            ]
        )

        f_v = np.squeeze(self.alpha[:, t_index - 1, :]).T * np.reshape(
            np.repeat(self.p_c[1, :], 3), (3, 8), "F"
        )
        f_v = np.flip(f_v[:, np.arange(1, 8)], 1)
        desired_state.vel = (
            np.array(
                [
                    [np.polyval(f_v[0, :], scale)],
                    [np.polyval(f_v[1, :], scale)],
                    [np.polyval(f_v[2, :], scale)],
                ]
            )
            / self.d0[t_index - 1]
        )

        f_a = np.squeeze(self.alpha[:, t_index - 1, :]).T * np.reshape(
            np.repeat(self.p_c[2, :], 3), (3, 8), "F"
        )
        f_a = np.flip(f_a[:, np.arange(2, 8)], 1)

        desired_state.acc = (
            np.array(
                [
                    [np.polyval(f_a[0, :], scale)],
                    [np.polyval(f_a[1, :], scale)],
                    [np.polyval(f_a[2, :], scale)],
                ]
            )
            / self.d0[t_index - 1] ** 2
        )

        dx = self.waypoints0[0, -1] - state[0, 0]
        dy = self.waypoints0[1, -1] - state[1, 0]

        _, _, cyaw = lqr.LQR.quat2rpy(state[3:7])

        if dx ** 2 + dy ** 2 < 1:
            desired_state.yaw = cyaw  # 0
        else:
            desired_state.yaw = cyaw  # -math.atan2(dy,dx)

        desired_state.yawdot = 0

        x0 = np.zeros((10, 1))
        x0[0:3] = np.array([desired_state.pos]).T
        x0[7:10] = desired_state.vel

        yaw = desired_state.yaw

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        roll = (1 / 9.8) * (desired_state.acc[0, 0] * sy - desired_state.acc[1, 0] * cy)
        pitch = (1 / 9.8) * (
            desired_state.acc[0, 0] * cy + desired_state.acc[1, 0] * sy
        )

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        x0[3:7] = np.array([lqr.LQR.rpy2quat(roll, pitch, yaw)]).T

        u0 = np.zeros((4, 1))

        omega_r = cr * cp * desired_state.yawdot
        u0[2] = omega_r

        R_BA = np.array(
            [
                [cr * cp - sr * sy * sp, -cr * sy, cy * sp + cp * sr * sy],
                [cp * sy + cy * sp * sr, cr * cy, sy * sp - cy * cp * sr],
                [-cr * sp, sr, cr * cp],
            ]
        )

        c = np.matmul(
            np.linalg.inv(R_BA), np.array([[0, 0, desired_state.acc[2, 0] + 9.8]]).T
        )
        u0[3] = c[2]

        return x0, u0


if __name__ == "__main__":


    seconds = 20
    points_per_second = 100
    total_points = seconds*points_per_second

    target_init_pos = 5*np.ones((3,1))
    target_pos = target_init_pos

    target_acc = np.array([[-0.25,0.25,0.25]]).T

    target_init_spd = 0
    target_init_vel = target_init_spd/math.sqrt(3) * np.ones((3,1))

    target_vel = target_init_vel
    target_pos_save = np.empty((0,3))
    target_vel_save = np.empty((0,3))

    x0 = np.zeros((10,1))


    fc = target_vel

    t = np.linspace(0, seconds, total_points)
    state_save = np.empty((0, 10))

    cur_state = np.zeros((10,1))
    cur_state[3,0] = 1

    update_period = 0.5


    for i in range(0, len(t)):
        if i % int(update_period*points_per_second) == 0:

            bias = target_vel*update_period + 0.5*target_acc*update_period**2
            goal = target_pos + bias

            waypoints = np.concatenate((x0[0:3], goal), 1)

            spd_gain = 2
            avg_spd = np.abs(target_vel + spd_gain*(x0[0:3]-target_pos))
            avg_spd = np.minimum(avg_spd[0], [10,10,10])

            ic = x0[7:10]
            fc = target_vel
            traj_generator = MinimumSnap(waypoints, ic, fc, avg_spd)

            prev_update_time = t[i]

        x0, u0 = traj_generator.compute(t[i]-prev_update_time, cur_state)
        
        state_save = np.append(state_save, x0.T, 0)

        target_pos = target_init_pos + target_init_vel*(t[i]) + 0.5*target_acc*(t[i])**2
        target_pos_save = np.append(target_pos_save, target_pos.T, 0)

        target_vel = target_init_vel + target_acc*(t[i])
        target_vel_save = np.append(target_vel_save, target_vel.T, 0)


    ax1 = plt.subplot(211)
    ax1.plot(t[:], state_save[:,0:3], t[:], target_pos_save)
    ax1.grid(color='k', linestyle='-', linewidth=0.25)
    ax1.legend(['x','y','z', 'tx', 'ty', 'tz'])
    ax1.set_title("Position [m]")

    ax2 = plt.subplot(212)
    ax2.plot(t[:], state_save[:,7:10], t[:], target_vel_save)
    ax2.grid(color='k', linestyle='-', linewidth=0.25)
    ax2.legend(['x','y','z', 'tx', 'ty', 'tz'])
    ax2.set_title("Velocity [m/s]")

    plt.show()
