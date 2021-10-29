import numpy as np
import math

max_iter = 30  # max iterations
dis_th = 5
tolerance = 0.001

class ICP():
    def __init__(self):
        pass

    def process(self,tar,src):
        laser_count = 0
        A = src.T[:,:2]
        B = tar.T[:,:2]
        src1 = np.ones((3,A.shape[0]))
        dst = np.ones((3,B.shape[0]))
        src1[:2,:] = np.copy(A.T)
        dst[:2,:] = np.copy(B.T)

        transform_acc = np.identity(3)
        prev_error = 0
        for i in range(max_iter):
            distances,indices = self.findNearest(src1[0:2,:].T,dst[0:2,:].T)
            T = self.getTransform(src1[0:2,:].T,dst[0:2,indices].T)
            src1 = np.dot(T,src1)
            mean_error = np.sum(distances)/distances.size
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error
        transform_acc = self.getTransform(A,src1[0:2,:].T)
        return transform_acc
    
    def findNearest(self,src,tar):
        indecies = np.zeros(src.shape[0], dtype=np.int)
        distances = np.zeros(src.shape[0])
        for i,s in enumerate(src):
            min_dist = np.inf
            for j,d in enumerate(tar):
                dist = np.linalg.norm(s-d)
                if dist < min_dist:
                    min_dist = dist
                    indecies[i] = j
                    distances[i] = dist
        return distances, indecies
    
    def getTransform(self,src,tar):
        assert len(src) == len(tar)
        centroid_src = np.mean(src,axis=0)
        centroid_tar = np.mean(tar,axis=0)
        AA = src - centroid_src
        BB = tar - centroid_tar
        W = np.dot(BB.T,AA)
        U,s,Vt = np.linalg.svd(W)
        R = np.dot(U,Vt)
        if np.linalg.det(R) < 0:
            Vt[2,:] *= -1
            R = np.dot(U,Vt)
        t = centroid_tar.T - np.dot(R,centroid_src.T)
        T = np.identity(3)
        T[0:2,0:2] = R
        T[0:2,2] = t
        return T

