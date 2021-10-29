#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import nanmax
import rospy
import time
from scipy.spatial.distance import pdist, squareform

class LandMarkSet():
    def __init__(self):
        self.position_x = []
        self.position_y = []
        self.id = []

class Extraction():
    def __init__(self):
        self.flag = 0
        # threshold of same cluster
        self.range_threshold = rospy.get_param('/extraction/range_threshold', 1.0)
        # threshold of certain obstacle as the feature of map
        self.radius_max_th = rospy.get_param('/extraction/radius_max_th', 0.3)
        self.landMark_min_pt = rospy.get_param('"/extraction/landMark_min_pt', 2)

    # Input: LaserScan, Output: MarkerArray
    def process(self, msg, trust = False):  # msg is laserToNumpy
        pc = msg
        pc = pc[:2, :]     # 2 * m
        m = pc.shape[1]

        num_cluster = 0
        labels = []
        same_labels = []
        is_obstacle = []

        for i in range(m-1):
            if np.linalg.norm([pc[0][i]-pc[0][i+1], pc[1][i]-pc[1][i+1]]) < self.range_threshold:
                labels.append(num_cluster)
                same_labels.append([pc[0][i], pc[1][i]])    # same_labels is n*2

            # judge whether the elements in the list are reasonable obstacles
            elif len(same_labels) >= 2: 
            #elif len(same_labels) >= self.landMark_min_pt:  
                labels.append(num_cluster)
                same_labels.append([pc[0][i], pc[1][i]]) 

                D = pdist(same_labels)
                D = squareform(D);  # D is n*n dist_matrix
                max_dist = nanmax(D) # calculate the max_dist
                if max_dist < self.radius_max_th: # is obstacle 
                    is_obstacle.append(num_cluster)

                num_cluster += 1    # new cluster
                same_labels = []

            # single point is obviously not an obstacle
            else:   
                labels.append(-1)

                num_cluster += 1
                same_labels = []    # ignore former operation record and renew
        landmarks = LandMarkSet()
        print "obstacle: ", is_obstacle

        #label is: [0, 0, 0, 1, 2, 2, 2, 2, -1, 3, 3...]

        if is_obstacle == []:
            self.flag = 1
            print "obstacle is [], break!"
            return 


        for i in range(len(labels)):
            if is_obstacle == []:
                break

            if labels[i] in is_obstacle:    # find all elements with the label of label[i]
                index = [j for j, x in enumerate(labels) if labels[j] == labels[i]]
                sum_x, sum_y = 0, 0
                is_obstacle.remove(labels[i])
                for k in index:
                    sum_x += pc[0][k]
                    sum_y += pc[1][k]
                ave_x, ave_y = sum_x/len(index), sum_y/len(index)

                landmarks.id.append(labels[i])
                landmarks.position_x.append(ave_x)
                landmarks.position_y.append(ave_y)

        return landmarks