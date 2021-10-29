import math
import numpy as np

# Covariance for EKF simulation
Q = np.diag([
    0.2,  # variance of location on x-axis
    0.2,  # variance of location on y-axis
    np.deg2rad(3.0)  # variance of yaw angle
]) ** 2  # predict state covariance
R = np.diag([0.2, 0.2,np.deg2rad(3.0)]) ** 2  # Observation x,y position covariance

DT = 0.1  # time tick [s]

class EKF():
    def __init__(self):
        pass
    def odom_model(self,x,u):
        # TODO
        return x

    def observation_model(self,x):
        # TODO
        return z

    def jacob_f(self,x, u):
        """
        Jacobian of Odom Model
        """
        # TODO
        return jF


    def jacob_h(self):
        # Jacobian of Observation Model
        # TODO
        return jH

    def estimate(self,xEst, PEst, z, u):
        #  Predict
        # TODO
        #  Update

        return xEst, PEst
