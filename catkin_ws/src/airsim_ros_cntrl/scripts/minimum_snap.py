#! /usr/bin/python3

import numpy as np
import rospy
import scipy.integrate as integrate
import math
from typing import Tuple

import matplotlib.pyplot as plt
import matplotlib.offsetbox as mob

import lqr

import airsim


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
    def __init__(self, waypoints: np.ndarray, initial_conditions: np.ndarray, final_conditions: np.ndarray, scaling_factor: float) -> None:
        """
        Constructs a minimum snap object from the desired waypoints

        Args:
            waypoints (np.ndarray): Desired Waypoints
        """
        self.t = rospy.get_time()
        self.state = np.zeros((10, 1))

        d = waypoints[:, 1:] - waypoints[:, 0:-1]

        #self.d0 = np.array([1/scaling_factor])
        
        #self.d0 = np.empty((3,0))
        #for i in range(0, d.shape[1]):
        #    segment_time = np.array([[d[0,i]/scaling_factor[0], d[1,i]/scaling_factor[1], d[2,i]/scaling_factor[2]]]).T
        #    self.d0 = np.append(self.d0, segment_time, 1)

        self.d0 = (
            np.sqrt(d[0, :] * d[0, :] + d[1, :] * d[1, :] + d[2, :] * d[2, :]) / scaling_factor
        )

        #self.traj_time = np.append(np.zeros((3,1)), np.cumsum(self.d0, 1), 1)
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

        b[8*N-6] = initial_conditions[0]*self.d0[0]
        b[8*N-5] = initial_conditions[1]*self.d0[0]
        b[8*N-4] = initial_conditions[2]*self.d0[0]
        b[8*N-3] = final_conditions[0]*self.d0[-1]
        b[8*N-2] = final_conditions[1]*self.d0[-1]
        b[8*N-1] = final_conditions[2]*self.d0[-1]

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
        
        #t = t*np.ones((3,1))
        #for i in range(0, t.shape[0]):
        #    if t[i] > self.traj_time[i, -1]:
        #        t[i] = self.traj_time[i, -1]
                

        '''
        t_index = np.zeros((3,1))        

        for j in range(0, 3):
            for i in range(0, len(self.traj_time[j])):
                if self.traj_time[j, i] >= t[i]:
                    t_index[j] = i

            if t_index[j] > 0:
                t[j] = t[j] - self.traj_time[t_index[j] -1]
        '''

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

        _, _, cyaw = airsim.to_eularian_angles(airsim.Quaternionr(state[4], state[5], state[6], state[3]))

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

        q = airsim.to_quaternion(pitch, roll, yaw).to_numpy_array()
        x0[3,0] = q[3]
        x0[4:7,0] = q[0:3]


        u0 = np.zeros((4, 1))

        omega_r = cr * cp * desired_state.yawdot

        if not compute_control:
            return x0, desired_state.acc

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

    rospy.init_node("minimum_snap_test")

    seconds = 30
    points_per_second = 100
    total_points = seconds*points_per_second

    target_init_pos = np.array([[0,0,0]], dtype='float64').T
    target_pos = np.copy(target_init_pos)

    target_init_vel = np.array([[0.0,0.0,0.5]], dtype='float64').T
    target_vel = np.copy(target_init_vel)

    target_pos_save = np.empty((0,3))
    target_vel_save = np.empty((0,3))

    x0 = np.zeros((10,1))
    x0[3] = 1

    fc = target_vel

    t = np.linspace(0, seconds, total_points)
    state_save = np.empty((0, 10))

    cur_state = np.zeros((10,1))
    cur_state[3,0] = 1


    ### Define Parameters ###

    horizon = 1.0
    spd_gain = 0.25
    profile = "sinusoidal"
    known_model = False

    ### ----------------- ###

    # Get differential change in acceleration
    def tax(t: float) -> float:
        if profile == "piecewise":
            if t < 5:
                ax = 0.5
            elif t < 10:
                ax = (-0.8-0.5)/(10-5)*(t-5)+0.5
            elif t < 15:
                ax = -0.8
            elif t < 21:
                ax = (0+0.8)/(21-15)*(t-15)-0.8
            elif t < 28:
                ax = (0.8-0.0)/(28-21)*(t-21)+0
            else:
                ax = 0.8
            return ax
        elif profile == "sinusoidal":
            return 0.1*np.cos(2*math.pi*0.1*t)
        elif profile == "constant":
            return 0.025
        elif profile == "zero":
            return 0.0
        else:
            raise Exception("Unkown acceleration profile" + str(profile)) 

    def tay(t: float) -> float:
        if profile == "piecewise":
            if t < 5:
                ay = 0.3
            elif t < 8:
                ay = (-0.1-0.3)/(8-5)*(t-5)+0.3
            elif t < 15:
                ay = -0.1
            elif t < 20:
                ay = (0+0.1)/(20-15)*(t-15)-0.1
            elif t < 28:
                ay = (0.5-0)/(28-20)*(t-20) + 0
            else:
                ay = 0.5

            return ay
        elif profile == "sinusoidal":
            return 0.1*np.sin(2*math.pi*0.1*t)
        elif profile == "constant":
            return 0.05
        elif profile == "zero":
            return 0.0
        else:
            raise Exception("Unkown acceleration profile" + str(profile))

    def taz(t: float) -> float:
        return 0.0

    def ta(t:float) -> float:
        return np.array([[tax(t),tay(t),taz(t)]]).T

    def integrate_velocity(t0, t1):
        vx,_ = integrate.quad(tax, t0, t1, epsabs=1.0e-4, epsrel=1.0e-4)
        vy,_ = integrate.quad(tay, t0, t1, epsabs=1.0e-4, epsrel=1.0e-4)
        vz,_ = integrate.quad(taz, t0, t1, epsabs=1.0e-4, epsrel=1.0e-4)
        return np.array([[vx,vy,vz]]).T

    def integrate_position(t0, t1, v0 = np.zeros((3,1))):
        px,_ = integrate.quad(lambda t:integrate.quad(tax, 0, t, epsabs=1.0e-4, epsrel=1.0e-4)[0], t0, t1, epsabs=1.0e-4, epsrel=1.0e-4)
        py,_ = integrate.quad(lambda t:integrate.quad(tay, 0, t, epsabs=1.0e-4, epsrel=1.0e-4)[0], t0, t1, epsabs=1.0e-4, epsrel=1.0e-4)
        pz,_ = integrate.quad(lambda t:integrate.quad(taz, 0, t, epsabs=1.0e-4, epsrel=1.0e-4)[0], t0, t1, epsabs=1.0e-4, epsrel=1.0e-4)
        p = np.array([[px,py,pz]]).T
        p += v0*(t1-t0)
        return p

    print("\n\n")
    print("Target acceleration profile: " + str(profile))
    print("Profile known by agent: " + str(known_model))
    print("")

    for i in range(0, len(t)):
        print("Time [s]: %5.2f/%5.2f" % (t[i],t[-1]), end="\r")
        if i % int(horizon*points_per_second) == 0:

            if known_model:
                bias = integrate_position(t[i], t[i]+horizon, target_vel)
                fv = target_vel + integrate_velocity(t[i], t[i]+horizon) 
                fa = ta(t[i]+horizon)
                fj = np.zeros((3,1))

            else:
                bias = target_vel*(horizon) + 0.5*ta(t[i])*(horizon)**2
                fv = target_vel + ta(t[i])*(horizon) 
                fa = ta(t[i])
                fj = np.zeros((3,1))

            goal = target_pos + bias

            waypoints = np.concatenate((x0[0:3], goal), 1)

            avg_spd = np.linalg.norm(target_vel) + spd_gain*np.linalg.norm((x0[0:3]-target_pos))
            avg_spd = np.minimum(avg_spd, 7)

            iv = x0[7:10]
            ia = np.zeros((3,1))
            ij = np.zeros((3,1))
            ic = np.concatenate([iv,ia,ij], 1).T

            fc = np.concatenate([fv,fa,fj], 1).T                

            traj_generator = MinimumSnap(waypoints, ic, fc, avg_spd)

            prev_update_time = t[i]
       

        x0, u0 = traj_generator.compute(t[i]-prev_update_time, cur_state)
        
        state_save = np.append(state_save, x0.T, 0)


        target_vel += integrate_velocity(t[i], t[i]+1/points_per_second)
        target_vel_save = np.append(target_vel_save, target_vel.T, 0)

        target_pos += integrate_position(t[i], t[i]+1/points_per_second, target_init_vel)
        target_pos_save = np.append(target_pos_save, target_pos.T, 0)

    print("\n")

    pos_rmse = np.sqrt(np.mean((target_pos_save-state_save[:,0:3])**2))
    vel_rmse = np.sqrt(np.mean((target_vel_save-state_save[:,7:10])**2))

    print("Position RMSE: " + str(pos_rmse))
    print("Velocity RMSE: " + str(vel_rmse))

    ax1 = plt.subplot(221)
    ax1.plot(t[:], state_save[:,0:3], t[:], target_pos_save)
    ax1.grid(color='k', linestyle='-', linewidth=0.25)
    ax1.legend(['x', 'y', 'z', 'tx', 'ty', 'tz'])
    ax1.set_title("Position over time")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position [m]")

    ax2 = plt.subplot(222)
    ax2.plot(state_save[:,0], state_save[:,1], target_pos_save[:,0], target_pos_save[:,1])
    ax2.grid(color='k', linestyle='-', linewidth=0.25)
    ax2.legend(['Agent', 'Target'])
    ax2.set_title("Position in the XY plane")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")

    ax3 = plt.subplot(223)
    ax3.plot(t[:], state_save[:,7:10], t[:], target_vel_save)
    ax3.grid(color='k', linestyle='-', linewidth=0.25)
    ax3.legend(['vx', 'vy', 'vz', 'vtx', 'vty', 'vtz'])
    ax3.set_title("Velocity over time")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Velocity [m/s]")

    ax4 = plt.subplot(224)
    ax4.plot(t, np.linalg.norm((target_pos_save-state_save[:,0:3]), axis=1), linestyle='-')
    ax4.plot(t, np.linalg.norm((target_vel_save-state_save[:,7:10]), axis=1), linestyle='--')
    ax4.grid(color='k', linestyle='-', linewidth=0.25)
    ax4.legend(['Position [m]', 'Velocity [m/s]'])
    ax4.set_title("Error over time")
    ax4.set_xlabel("t")
    ax4.set_ylabel("e")

    annotation = "Profile: " + profile + "\nKnown: " + str(known_model) + "\nHorizon [s]: " + str(horizon) + "\nSpeed gain: " + str(spd_gain) + "\nPos. RMSE: {prmse:5.3f}\nVel. RMSE: {vrmse:5.3f}"
    at = mob.AnchoredText(annotation.format(prmse=pos_rmse, vrmse=vel_rmse), prop=dict(size=10), frameon=True, borderpad=0, loc='center')
    at.patch.set_boxstyle("round, pad=0.5, rounding_size=0.2")
    
    ax4.add_artist(at)
    #ax4.annotate(annotation.format(prmse=pos_rmse, vrmse=vel_rmse), xy=(15,4), bbox=)

    plt.show()
