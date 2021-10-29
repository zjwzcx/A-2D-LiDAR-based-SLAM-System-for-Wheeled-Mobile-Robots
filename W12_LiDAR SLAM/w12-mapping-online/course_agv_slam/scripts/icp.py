#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class ICP:
    def __init__(self):
        self.laser_count = 0
        # robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        # sensor states = robot_x_y_theta
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]

        # max iterations
        self.max_iter = rospy.get_param('/icp/max_iter', 30)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th',5)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance', 0.001)
        # if is the first scan, set as the map/target
        self.isFirstScan = True
        # src point cloud matrix
        self.src_pc = []
        # target point cloud matrix
        self.tar_pc = []
        #self.transform_acc = []

        #self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.odom_pub = rospy.Publisher('icp_odom', Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def process(self, tar_pc, src_pc):

        tolerance = rospy.get_param('/icp/tolerance', 0.001)

        B = tar_pc[:2, :]    # target point_cloud, 3*100, but the third row is [1, 1, ...]
        tar_pc = np.ones((3, B.shape[1]))
        tar_pc[:2, :] = np.copy(B)


        A = src_pc[:2, :]     # source point_cloud, 3*100, but the third row is [1, 1, ...]
        src_pc = np.ones((3, A.shape[1]))
        src_pc[:2, :] = np.copy(A)  # src/tar is 3*300 nparray

        m = src_pc.shape[1]    # m = 300

        # init some variables
        pre_error = 0
        iter_cnt = 0
        '''
        A/B is 3*300 nparray, but all elements in the third rows of them is 1, so cut A/B into 2*300 nparray.


        A: 2 * 300
        B: 2 * 300
        self.src_pc: 3 * 300
        self.tar_pc: 3 * 300
        '''
        for i in range(self.max_iter):
            # src is previous_points, tar is current_points
            distances, indices = self.findNearest(src_pc[:2, :].transpose(), tar_pc[:2, :].transpose())
            
            T = self.getTransform(src_pc[:2, :].transpose(), tar_pc[:2, indices].transpose())

            src_pc = np.dot(T, src_pc)

            iter_cnt += 1   # cnt: connected

            mean_error = np.sum(distances)/distances.size
            if abs(pre_error - mean_error) < tolerance:
                break
            pre_error = mean_error
        # A.transpose() = 300 * 2

        T = self.getTransform(A.transpose(), src_pc[:2, :].transpose())
        #self.transform_acc = np.dot(self.transform_acc, T)
        #print("total_iter: ",iter_cnt)
        #self.tar_pc = self.laserToNumpy(msg)    # [x, y, theta]
        #self.publishResult(T)
        #time_1 = rospy.Time.now()
        #print("time_cost: ",time_1-time_0)
        return T

    def findNearest(self, src, tar):
        '''
        src: 300 * 2 
        tar: 300 * 2
        '''

        indices = np.zeros(src.shape[0], dtype = np.int)
        distances = np.zeros(src.shape[0])

        for i, s in enumerate(src):
            min_dist = np.inf
            for j, d in enumerate(tar):
                dist = np.linalg.norm(s-d)
                if dist < min_dist:
                    min_dist = dist
                    indices[i] = j
                    distances[i] = dist
            '''
            if i == j:  # case: src > tar
                distances = distances[:i]
                indices = indices[:i]
                break
            '''

        return distances, indices

    def match(self, map_z, laser_z):
        new_map_z = []
        indices = np.zeros(len(map_z), dtype = np.int)
        distances = np.zeros(len(map_z))

        n1 = len(laser_z)/2
        n2 = len(map_z)/2
        for i in range(n1):
            x1 = laser_z[2*i]
            y1 = laser_z[2*i+1]
            min_dist = np.inf
            print "x1, y1:", x1, y1
            for j in range(n2):
                x2 = map_z[2*j]
                y2 = map_z[2*j+1]
                dist = np.linalg.norm([y2-y1,x2-x1])
                if dist < min_dist:
                    min_dist = dist
                    indices[i] = j
                    distances[i] = dist
            print "x2, y2:",[map_z[2*indices[i]]], [map_z[2*indices[i]+1]]
            #print "min_dist", min_dist

            new_map_z.append([map_z[2*indices[i]]])
            new_map_z.append([map_z[2*indices[i]+1]])
            #print "list new_map_z is: ", new_map_z
            new_map_z = np.array(new_map_z)
            #print "array new_map_z is:", new_map_z

        return new_map_z    # 2n*1, self.z1



    def getTransform(self, src, tar):
        '''
        src: 300 * 2
        tar: 300 * 2
        '''
        centroid_A = np.mean(src, axis = 0)
        centroid_B = np.mean(tar, axis = 0)

        AA = src - centroid_A
        BB = tar - centroid_B

        W = np.dot(BB.transpose(), AA)  # 2*2
        U, S, Vt = np.linalg.svd(W)
        R = np.dot(U, Vt)

        if np.linalg.det(R) < 0:
            print "========================"
            print "it could be executed!!"
            print "========================"
            Vt[1, :] *= -1
            R = np.dot(U, Vt)

        #t = centroid_T.reshape(2, -1) - np.dot(R, (centroid_T.reshape(2, -1).transpose()))
        t = centroid_B.transpose() - np.dot(R, centroid_A.transpose())

        T = np.identity(3)
        T[:2, :2] = R
        T[0, 2] = t[0]
        T[1, 2] = t[1]

        return T    # T: 3*3

    def publishResult(self, T):     # change into a coordinate system transformer
        # T: 3*3
        #print "T in publishResult is: "
        #print T
        delta_yaw = math.atan2(T[1,0],T[0,0])
        print "sensor-delta-xyt: ",T[0,2],T[1,2],delta_yaw
        s = self.sensor_sta
        self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        self.sensor_sta[2] = s[2] + delta_yaw
        print "sensor-global: ",self.sensor_sta

        # tf
        s = self.sensor_sta
        q = tf.transformations.quaternion_from_euler(0, 0, self.sensor_sta[2])
        self.odom_broadcaster.sendTransform((s[0], s[1], 0.001),(q[0], q[1],q[2],q[3]),
                            rospy.Time.now(), "icp_odom", "world_base")

        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        #return odom

    def laserToNumpy(self, msg):
        '''
        [[x1, y1, theta1],
        [x2, y2, theta2],
            ......
        [xn, yn, thetan]]
        change to column
        '''
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

def main():
    rospy.init_node('icp_node')
    icp = ICP()
    rospy.spin()

if __name__ == '__main__':
    main()