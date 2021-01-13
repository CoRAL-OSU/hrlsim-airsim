#! /usr/bin/python2

import time
import multiprocessing as mp
import numpy as np
import math
import sys

import airsim
import slycot
import control

class LQR:
    def __init__(self):
        self.Q = np.diag([50,50,100,1,1,1,1,2,2,2])
        self.R = np.diag([50,50,50,4])

        self.A = np.zeros((10,10))
        self.B = np.zeros((10,4))
        self.K = np.zeros((10,10))

        self.x0 = np.zeros((10,1))
        self.u0 = np.zeros((4,1))


    def set_costs(self, Q=None, R=None):
        if Q != None:
            assert len(Q) == 10, "Q must be a list of length 10"
            self.Q = np.diag(Q)

        if R != None:
            assert len(R) == 4, "R must be a list of length 4"
            self.R = np.diag(R)

    def set_goals(self, x0=None, u0=None):
        if type(x0) != None:
            assert x0.shape == (10,1), "x0 must be a 10x1 vector"
            self.x0 = x0
        if type(u0) != None:
            assert u0.shape == (4,1), "u0 must be a 4x1 vector"
            self.u0 = u0


    def updateGains(self, x, u):
        assert np.shape(x) == (10,1), "The state must be a 10x1 state vector"
        assert np.shape(u) == (4,1), "The goal command must be a 4x1 control vector"

        for i in range(0,4):       
            if abs(u[i]) < 0.001:
                u[i] = 0.001

        self.A = self.__updateA(x,u)
        self.B = self.__updateB(x)

        Co = control.ctrb(self.A,self.B)

        try:
            assert np.linalg.matrix_rank(self.A) <= np.linalg.matrix_rank(Co), "System is not controllable"
        
        except AssertionError:
            print("System is not controllable")
            exit(1)     

        self.K,_,_ = control.lqr(self.A,self.B,self.Q,self.R)   

        return self.K            


    def computeControl(self, x):
        assert np.shape(x) == (10,1), "The initial condition must be a 10x1 state vector"
        
        u = np.zeros((4,1))

        # Compute the optimal control step
        #print(str(x.T))
        #print(str(self.x0.T))
        u = self.u0 - np.matmul(self.K,(x-self.x0))

        return u


    @staticmethod
    def quat2rpy(q):
        roll    = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1])*q[1]+q[2]*q[2])
        pitch   = math.asin(2*(q[0]*q[2]-q[3]*q[1]))
        yaw     = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
        return roll, pitch, yaw

    @staticmethod
    def rpy2quat(roll, pitch, yaw):
        cr = math.cos(roll/2)
        sr = math.sin(roll/2)
        cp = math.cos(pitch/2)
        sp = math.sin(pitch/2)
        cy = math.cos(yaw/2)
        sy = math.sin(yaw/2)
        q = [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy - sr*sp*cy]     
        return q  

    @staticmethod
    def ned2xyz(x):
        assert np.shape(x) == (10,1), "The state must be a 10x1 vector"
        #tp = np.matrix([[0,1,0],[1,0,0],[0,0,-1]])
        #tq = np.block([[1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,-1]])

        #T = np.block([[ tp, np.zeros((3,4)), np.zeros((3,3)) ],
        #              [ np.zeros((4,3)), tq, np.zeros((4,3)) ],
        #              [ np.zeros((3,3)), np.zeros((3,4)), tp ]])

        xnew = np.zeros((10,1))

        xnew[0] = x[1]
        xnew[1] = x[0]
        xnew[2] = -x[2]
        
        # Apply quaternion rotation to  translate. Don't swap axis
        # R =[1 0 0 ; 0 -1 0; 0 0 -1]
        q1 = airsim.Quaternionr(x[4], x[5], x[6], x[3])
        q_rot = airsim.Quaternionr(math.sqrt(2)/2, math.sqrt(2)/2, 0, 0)

        qnew = q1.rotate(q_rot)
        qnew = qnew.to_numpy_array()

        xnew[3] = qnew[3]
        xnew[4] = qnew[0]
        xnew[5] = qnew[1]
        xnew[6] = qnew[2]
        
        xnew[7] = x[8]
        xnew[8] = x[7]
        xnew[9] = -x[9]

        x = xnew#np.matmul(T,x)
        return x

    @staticmethod
    def get_state(x):
        assert np.shape(x) == (10,1), "The state must be a 10x1 vector"
        return x[0:3], x[3:7], x[7:10]
    
    @staticmethod
    def get_command(u):
        assert np.shape(u) == (4,1), "The command must be a 4x1 vector"
        return u[0:3], u[3][0]

    @staticmethod
    def set_state(p, q, v):
        assert len(p) == 3, "The position must be a 3x1 list"
        assert len(q) == 4, "The quaternion must be a 4x1 list"
        assert len(v) == 3, "The velocity must be a 3x1 list"

        p = np.array([p], dtype='float').T
        q = np.array([q], dtype='float').T
        v = np.array([v], dtype='float').T

        x = np.block([ [p], 
                       [q], 
                       [v] ])
        return x

    @staticmethod
    def set_command(omega, c):
        assert len(omega) == 3, "The body rates must be a 3x1 list"
        
        omega = np.array([omega], dtype='float').T

        u = np.block([ [omega],
                       [c]     ])
        return u



    def __updateA(self, x, u):
        p,q,v = self.get_state(x)
        omega,c = self.get_command(u)

        # 10x10 block matrix
        A = np.block( [[ np.zeros((3,3)), np.zeros((3,4)),      np.identity(3)  ],
                       [ np.zeros((4,3)), LQR.__qdotq(omega,q), np.zeros((4,3)) ],
                       [ np.zeros((3,3)), LQR.__vdotq(c,q),     np.zeros((3,3)) ]])
        return A

    def __updateB(self, x):
        p,q,v = LQR.get_state(x)

        # 10x4 block matrix
        B = np.block( [[ np.zeros((3,3)),    np.zeros((3,1)) ],
                       [ LQR.__qdotomega(q), np.zeros((4,1)) ],
                       [ np.zeros((3,3)),    LQR.__vdotc(q)  ]])
        return B

    @staticmethod
    def __qdotq(omega, q):
        y = np.block( [[ 0.0,       -omega[0],  -omega[1],   -omega[2] ],
                       [ omega[0],   0.0,        omega[2],   -omega[1] ],
                       [ omega[1],  -omega[2],   0.0,         omega[0] ],
                       [ omega[2],   omega[1],  -omega[0],    0.0        ]] )
        
        qnorm = np.linalg.norm(q)
        return 0.5*np.matmul(y,(np.identity(4) - (math.pow(qnorm,-2)*q*q.T))/qnorm)

    @staticmethod
    def __vdotq(c, q):
        y = np.block( [[ q[2], q[3], q[0], q[1] ],
                       [-q[1],-q[0], q[3], q[2] ],
                       [ q[0],-q[1],-q[2], q[3] ]] )

        qnorm = np.linalg.norm(q)

        return 2.0*c*np.matmul(y,(np.identity(4) - (math.pow(qnorm,-2)*q*q.T))/qnorm)

    @staticmethod     
    def __qdotomega(q):
        y = np.block( [[-q[1],-q[2],-q[3] ],
                       [ q[0],-q[3],-q[2] ],
                       [ q[3], q[0], q[1] ],
                       [-q[2], q[1], q[0] ]])
        return 0.5*y

    @staticmethod
    def __vdotc(q):
        y = np.block( [[ q[0]*q[2] + q[1]*q[3] ],
                       [ q[2]*q[3] - q[0]*q[1] ],
                       [ q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3] ]])
        return y


class LowLevelController:

    def __init__(self, drone_name, sim_lock, sim_client):
        """
        Initialize a Drone process, spinup the ROS node, and setup topics/services/action servers
        
        All topic names are set up as /swarm_name/drone_name/major_cmd/minor_cmd. Eg. commanding the velocity
        of drone1 in swarm1 would be: /swarm1/drones1/cmd/vel

        @param drone_name (string) Name of drone
        ---
        @param sim_lock (mp.Lock) Multiprocessing lock for sim client 
        ---
        @param sim_client (airsim.MultirotorClient) Airsim Python Client
        ---
        """
        self.__drone_name = drone_name
        self.__client = sim_client
        self.__client_lock = sim_lock

        self.__rate = 1.0/10.0   # 10 Hz


    def set_pwm(self, pwms):
        '''
        pwms[0] -> front right
        pwms[1] -> back left
        pwms[2] -> front left
        pwms[3] -> back right
        '''

        with self.__client_lock:
            self.__client.moveByMotorPWMsAsync(pwms[0], pwms[1], pwms[2], pwms[3], self.__rate*2, self.__drone_name)




if __name__ == "__main__":
    controller = LQR()

    p = [0,0,0]
    q = [1,0,0,0]
    v = [0,0,0]
    init_state = LQR.set_state(p, q, v)

    p = [1,1,1]
    q = [1,0,0,0]
    v = [0,0,0]
    x0 = LQR.set_state(p, q, v)

    omega = [0,0,0]
    c = 9.8
    u0 = LQR.set_command(omega, c)

    u = controller.compute(x0, u0, init_state, 1)

    print(u)