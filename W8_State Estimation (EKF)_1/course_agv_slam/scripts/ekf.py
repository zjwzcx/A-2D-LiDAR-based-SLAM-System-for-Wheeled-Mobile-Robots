import math
import copy
import numpy as np

# Covariance Matrix of prediction
Q = np.diag([
    0.2,  # variance of location on x-axis
    0.2,  # variance of location on y-axis
    math.pi/60
]) ** 2  # predict state covariance

# Covariance Matrix of observation
R = np.diag([0.2, 0.2, math.pi/60]) ** 2  # Observation x,y position covariance

DT = 0.1

class EKF():
    def __init__(self):
        pass
    def odom_model(self, x, T):
        # xPre_{t} = g(x_{t-1}, u_{t-1})
        # x: 3*1
        delta_yaw = math.atan2(T[1, 0], T[0, 0])
        s = copy.deepcopy(x)
        s[0] = x[0] + math.cos(x[2])*T[0,2] - math.sin(x[2])*T[1,2]
        s[1] = x[1] + math.sin(x[2])*T[0,2] + math.cos(x[2])*T[1,2]
        s[2] = x[2] + delta_yaw

        return s

    def observation_model(self, x):
        # H: 3*3, x: 3*1
        H = np.eye(3)
        z = np.dot(H, x)

        return z

    def jacob_f(self,x, T): # dg/dx, that is g1
        """
        Jacobian of Odom Model
        """
        jF = np.eye(3)
        cos = T[0, 0]
        sin = T[1, 0]
        m = T[0, 2]
        n = T[1, 2]
        jF[0, 2] = -m * sin - n * cos
        jF[1, 2] = m * cos -n * sin
        return jF


    def jacob_h(self):  # dg/du, that is g2
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        return jH

    def estimate(self, xEst, PEst, z, T):   # 
        '''
        xEst is the estimating state vector of x_{t-1}, 4*1
        PEst is covariace matrix of the state, 4*4
        z is the observation of absolute postion
        u is delta_x
        odom_model(): g()
        observation_model(): m()
        '''
        # Predict

        xPred = self.odom_model(xEst, T) # x_{t} = g(xEst, u), xPred: 3*3
        jF = self.jacob_f(xEst, T)   # g1, jF: 3*3
        PPred = np.dot(np.dot(jF, PEst), jF.transpose()) + R    # 3*3

        # Update
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)    # zPre = m(xPre, 0)
        y = z - zPred   # y = Zt - m(xPre, 0)

        # Melting Prediction
        S = np.dot(np.dot(jH, PPred), jH.transpose()) + Q
        K = np.dot(np.dot(PPred, jH.transpose()), np.linalg.inv(S))
        xEst = xPred + np.dot(K, y) # xEst = xPre + K * (Zt - m(xPre, 0))
        PEst = np.dot((np.eye(3) - np.dot(K, jH)), PPred)
        
        return xEst, PEst
