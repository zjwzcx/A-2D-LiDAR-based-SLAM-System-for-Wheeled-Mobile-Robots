import math
import copy
import numpy as np

# Covariance Matrix of prediction
#Q = np.diag([
#    0.2,  # variance of location on x-axis
#    0.2,  # variance of location on y-axis
#    math.pi/60
#]) ** 2  # predict state covariance

# Covariance Matrix of observation
R = np.diag([0.04, 0.04, math.pi**2/3600]) # Observation x,y position covariance

DT = 0.1

class EKF():
    def __init__(self):
        self.n = 0
        pass
    def odom_model(self, x, u):
        # xPre_{t} = g(x_{t-1}, u_{t-1})
        # x: 1*3
        # u: 3*3
        s = np.zeros([3, 1])   # s: 3*1
        s[0] = x[0] + math.cos(x[2])*u[0] - math.sin(x[2])*u[1]
        s[1] = x[1] + math.sin(x[2])*u[0] + math.cos(x[2])*u[1]
        s[2] = x[2] + u[2]

        return s

    def jacob_f(self,x, u): # dg/dx, that is g1
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        """
        #calculate diffential state
        jF = np.array([[0.0, 0.0, (-u[0]*math.sin(x[2])-u[1]*math.cos(x[2]))[0]],
                    [0.0, 0.0, (u[0]*math.cos(x[2])-u[1]*math.sin(x[2]))[0]],
                    [0.0, 0.0, 0.0]])
        return jF

    def jacob_h(self, xEst, z):  # dg/du, that is g2
        '''
        Jacobian of Observation Model
        '''
        jH = np.ones([2*self.n, 3])
        for i in range(self.n):
            theta = xEst[2]
            jH[2*i][0] = -math.cos(theta)
            jH[2*i][1] = -math.sin(theta)
            jH[2*i][2] = z[2*i+1]
            jH[2*i+1][0] = math.sin(theta)
            jH[2*i+1][1] = -math.cos(theta)
            jH[2*i+1][2] = -z[2*i]
        return jH   # 2n*3

    def estimate(self, xEst, PEst, z1, z2, u):   # 
        '''
        xEst is the state vector of x_{t-1}, 1*3/3*1
        PEst is covariace matrix of the state, 3*3
        z1: 2n*1
        z2: 2n*1
        u is the differences of state, 1*3/3*1

        odom_model(): g()
        observation_model(): m()
        '''
        #print "xEst is: ", xEst
        print "z1 is: ", z1 # 1*2
        print "z2 is: ", z2 # 2*1
        #print "u is: ", u
        self.n = len(z2)/2
        Q = [0.2] * len(z2)
        
        # Predict
        xPred = self.odom_model(xEst, u) # x_{t} = g(xEst, u), xPred: 3*3
        jF = self.jacob_f(xEst, u)   # g1, jF: 3*3
        PPred = np.dot(np.dot(jF, PEst), jF.transpose()) + R    # 3*3


        # Update
        jH = self.jacob_h(xEst, z2) # 2n*3
        y = z2 - z1 # 2n*1


        # Melting Prediction
        S = np.dot(np.dot(jH, PPred), jH.transpose()) + Q   # 2n*2n
        K = np.dot(np.dot(PPred, jH.transpose()), np.linalg.inv(S)) # 3*2n

        #print "K IS: ", K
        #print "y is: ", y       # y should be 2*1 instead of 2*2
        #print "Ky is: ", np.dot(K, y)
        #print "xPred is: ", xPred

        xEst = xPred + np.dot(K, y) # xEst = xPre + K * (Zt - m(xPre, 0)), 3*1
        #print"K IS:", K
        #print"jH is:", jH
        PEst = np.dot((np.eye(3) - np.dot(K, jH)), PPred)   # 3*3
        
        return xEst, PEst
