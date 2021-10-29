import math
import numpy as np

# EKF state covariance
Cx = np.diag([0.35, 0.35, np.deg2rad(15.0)]) ** 2
# Q = np.diag([0.1, np.deg2rad(0.666)]) ** 2
# R = np.diag([0.5, np.deg2rad(6.66)]) ** 2
M_DIST_TH = 0.6  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

class EKF():
    def __init__(self):
        pass
    def estimate(self, xEst, PEst, z, u):
        '''
        init_xEst: 3*1 --> (3+2n)*1
        init_PEst: 3*3 --> (3+2n)*(3+2n)
        z: n*3
        u: 3*1
        '''
        n = len(z)
        n1 = len(xEst)  # 2n+3

        # Predict
        Fx = np.hstack((np.eye(3), np.zeros((3, n1-3))))    # 3*(3+2n)
        jF = self.jacob_f(xEst[:3], u)  # 3*3
        G_temp = np.dot(np.dot(Fx.transpose(), jF), Fx) # (2n+3)*(2n+3)
        G = np.eye(3) + G_temp[:3, :3]  # 3*3
        xEst[:3] = self.motion_model(xEst[:3], u)
        PEst[:3, :3] = np.dot(np.dot(G.transpose(), PEst[:3, :3]), G) + np.dot(np.dot(Fx.transpose(), Cx), Fx)[:3, :3]
        '''
        Fx = np.hstack((np.eye(3), np.zeros((3, n1-3))))  # 3*(3+2n)
        xPred = self.odom_model(xEst, Fx, u)    # (2n+3)*3

        Gt = np.eye(n1) + np.dot(np.dot(Fx.transpose(), self.jacob_f(xEst, u)), Fx)  # 3*3
        PPred = np.dot(np.dot(Gt.transpose(), PEst), Gt)    # 3*3
        PPred[:3, :3] += Cx # add noise

        xEst = xPred
        PEst = PPred
        '''

        # Update
        for iz in range(n):
            num_lm = (n1-3)/2
            min_id = self.search_correspond_landmark_id(xEst, PEst, z[iz, :2])

            if num_lm == min_id:    # new landmark
                xEst = np.vstack((xEst, self.calc_landmark_position(xEst, z[iz, :])))
                PEst = np.vstack((np.hstack((PEst, np.zeros((n1, 2)))), np.hstack((np.zeros((2, n1)), np.eye(2)))))

            lm = self.get_landmark_position_from_state(xEst, min_id)
            y, S, H = self.laser_correction(lm, xEst, PEst, z[iz, 0:2], min_id)
            K = np.dot(np.dot(PEst, H.transpose()), np.linalg.inv(S))
            xEst = xEst + np.dot(K, y)
            PEst = np.dot((np.eye(len(xEst) - np.dot(K, H)), PEst)
        # normalization
        xEst[2] = self.pi_2_pi(xEst[2])

        return xEst, PEst

    def motion_model(self, x, u):
        """
        x = [x,y,w,...,xi,yi,...]T, (2n+3)*1
        u = [ox,oy,ow]T, 3*1
        """
        s = np.zeros((3,1))
        s[0] = x[0] + math.cos(x[2])*u[0] - math.sin(x[2])*u[1]
        s[1] = x[1] + math.sin(x[2])*u[0] + math.cos(x[2])*u[1]
        s[2] = x[2] + u[2]
        x[:3] = s

        return x[:3]    # (3+2n)*1

    def jacob_f(self, x, u): # dg/dx, that is g1
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        """
        #calculate diffential state
        jF = np.array([[0.0, 0.0, (-math.sin(x[2])*u[0]-math.cos(x[2])*u[1])[0]],
                    [0.0, 0.0, (math.cos(x[2])*u[0]-math.sin(x[2])*u[1])[0]],
                    [0.0, 0.0, 0.0]])
        return jF

    def calc_landmark_position(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

        return zp

    def get_landmark_position_from_state(self, x, ind):
        lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

        return lm

    def search_correspond_landmark_id(self, xAug, PAug, zi):
        """
        Landmark association with Mahalanobis distance
        """
        nLM = (len(xAug)-3)/2
        min_dist = []

        # TODO search and update min_dist
        for i in range(nLM):
            lm = self.get_landmark_position_from_state(xAug, i)
            y, S, H = self.laser_correction(lm, xAug, PAug, zi, i)
            dist = np.dot(np.dot(y.transpose(), np.linalg.inv(S)), y)
            min_dist.append(dist)

        min_dist.append(M_DIST_TH)  # new landmark
        min_id = min_dist.index(min(min_dist))
        # if nLM == min_id:
        return min_id

    def laser_correction(self, lm, xEst, PEst, z, LMid):   
        delta = lm - xEst[:2]   # distance between landmarks and our robot
        q = np.dot(delta.T, delta)[0][0]    # square of distance 
        z_angle = math.atan2(delta[1][0], delta[0][0]) - xEst[2][0]
        zi = np.array([[math.sqrt(q), self.pi_2_pi(z_angle)]])

        y = (z - zi).T
        y[1] = self.pi_2_pi(y[1])    # normalization

        H = self.jacob_h(q, delta, xEst, LMid + 1)  # high-dimension case
        S = np.dot(np.dot(H, PEst), H.T) + Cx[:2, :2]

        return y, S, H

    def jacob_h(self, q, delta, x, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                    [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

        G = G/q
        nLM = (len(x)-3)/2
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

        F = np.vstack((F1, F2))
        H = np.dot(G, F)

        return H

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
