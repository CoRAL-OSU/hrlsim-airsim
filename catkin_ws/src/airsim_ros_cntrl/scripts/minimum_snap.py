#! /usr/bin/python2

import time
import multiprocessing as mp
import numpy as np
import math
import sys

import airsim
import slycot
import control


class MinimumSnap:
    def __init__(self, waypoints):
        self.t = time.time()
        self.state = np.zeros((10,1))
        
        d = waypoints[:,1:] - waypoints[:,0:-1]
        self.d0 = 2*np.sqrt(d[0,:]*d[0,:] + d[1,:]*d[1,:] + d[2,:]*d[2,:])
        self.traj_time = np.array([0, np.cumsum(self.d0)])
        self.waypoints0 = waypoints

        N = np.size(waypoints, 1)-1

        self.p_c = np.zeros((7,8))
        self.p_c[0,:] = np.ones((1,8))

        p = np.poly1d(self.p_c[0,:])

        p_d1 = np.polyder(p)
        self.p_c[1,1:] = np.flip(p_d1.c)

        p_d2 = np.polyder(p_d1)
        self.p_c[2,2:] = np.flip(p_d2.c)

        p_d3 = np.polyder(p_d2)
        self.p_c[3,3:] = np.flip(p_d3.c)

        p_d4 = np.polyder(p_d3)
        self.p_c[4,4:] = np.flip(p_d4.c)

        p_d5 = np.polyder(p_d4)
        self.p_c[5,5:] = np.flip(p_d5.c)

        p_d6 = np.polyder(p_d5)
        self.p_c[6,6:] = np.flip(p_d6.c)

        head_c = np.diag(self.p_c)
        head_c = np.diag(head_c)

        head_c = np.append(head_c, np.zeros((7,1)), 1)


        A = np.zeros((8*N,8*N))
        b = np.zeros((8*N, 3))

        for i in range(1,N):

            A[(i-1)*8, np.arange(0,8)+(i-1)*8] = head_c[0,:]
            b[(i-1)*8, :] = waypoints[:,i].T

            A[(i-1)*8+1, np.arange(0,8)+(i-1)*8] = self.p_c[0,:]
            b[(i-1)*8+1, :] = waypoints[:,i].T

            if i < N:
                A[(i-1)*8+2, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[1,:], -head_c[1,:])
                A[(i-1)*8+3, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[2,:], -head_c[2,:])
                A[(i-1)*8+4, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[3,:], -head_c[3,:])
                A[(i-1)*8+5, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[4,:], -head_c[4,:])
                A[(i-1)*8+6, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[5,:], -head_c[5,:])
                A[(i-1)*8+7, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[6,:], -head_c[6,:])


        A[8*N-6, np.arange(0,8)] = head_c[1,:]
        A[8*N-5, np.arange(0,8)] = head_c[2,:]
        A[8*N-4, np.arange(0,8)] = head_c[3,:]
        A[8*N-3, np.arange(0,8)+8*(N-1)] = self.p_c[1,:]
        A[8*N-2, np.arange(0,8)+8*(N-1)] = self.p_c[2,:]
        A[8*N-1, np.arange(0,8)+8*(N-1)] = self.p_c[3,:]

        np.savetxt("A.csv", A, delimiter=",")

        x1 = np.linalg.inv(A)*b[:,0]    
        x2 = np.linalg.inv(A)*b[:,1]
        x3 = np.linalg.inv(A)*b[:,2]
        
        self.alpha = np.zeros((x2.shape, 8, N))

        print(self.alpha.shape)
        self.alpha[:,:,1] = np.reshape(x1, 8, N)
        self.alpha[:,:,2] = np.reshape(x2, 8, N)
        self.alpha[:,:,3] = np.reshape(x3, 8, N)



if __name__ == "__main__":

    waypoints = np.array(  [[0,    0,   0],
                            [1,    1,   1],
                            [3,    5,   0],
                            [20,   20,  2],
                            [6,    5,   1],
                            [0,    0,   0]]).T

    traj_generator = MinimumSnap(waypoints)