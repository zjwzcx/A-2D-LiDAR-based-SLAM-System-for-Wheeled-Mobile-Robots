#!/usr/bin/env python
import rospy
import tf
import math
import random
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker   # Landmark
import numpy as np
from icp import ICP
from ekf import EKF
from extraction import LandMarkSet, Extraction  # Extract landmark
import sys
from copy import deepcopy 

class Localization():
    def __init__(self):
        self.icp = ICP()
        self.ekf = EKF()
        self.extraction = Extraction()
        self.laser_count = 0
        
        # odom robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []
        self.pc_laser2numpy = []
        self.z1 = []
        self.z2 = []
        self.map_pc = []
        self.new_map_pc = []
        self.flag = 0
        # State Vector [x y yaw]
        self.xOdom = [0,0,0]
        self.xEst = [0,0,0]
        self.PEst = np.eye(3)
        self.origin_x = 0
        self.origin_y = 0
        self.resolution = 0
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.updateMap)
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.landMark_pub = rospy.Publisher('/landMarks', MarkerArray, queue_size=1)
        self.publishResult_est()

    def updateMap(self, msg):
        print('start getting map')
        self.map = msg
        # Update for planning algorithm
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        tx,ty = np.nonzero((map_data > 20)|(map_data < -0.5))
        ox = (tx*self.map.info.resolution+self.map.info.origin.position.x)*1.0
        oy = (ty*self.map.info.resolution+self.map.info.origin.position.y)*1.0
        self.obstacle = np.vstack((ox,oy))   #transpose()
        self.obstacle_r = self.map.info.resolution
        print("debug: update map obstacle success! ")
        self.width = msg.info.width
        self.height = msg.info.height
        raw = np.array(msg.data, dtype = np.int8)
        self.map = raw.reshape((self.height, self.width))
        for i in range(self.width):
            for j in range(self.height):
                if self.map[i][j] == -1:
                    if self.map[i-1][j-1]==0 and self.map[i-1][j]==0 and self.map[i-1][j+1]==0 and self.map[i][j-1]==0 and self.map[i][j+1]==0 and self.map[i+1][j-1]==0 and self.map[i+1][j]==0 and self.map[i+1][j+1]==0:
                        self.map_pc.append(i)
                        self.map_pc.append(j) # 2n*1
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution

        self.new_map_pc = self.mapToworld(self.map_pc, self.resolution, self.origin_x, self.origin_y)
        #print self.new_map_pc

    def laserCallback(self, msg):
        #if self.laser_count == 0:
            #self.time_start = rospy.get_time()
            #print('start time',self.time_start)
        self.laser_count += 1
        if self.laser_count <= 5:
            return
        self.laser_count = 0
        self.pc_laser2numpy = self.laserToNumpy(msg)

        # Observation Model
        u_obser = self.calc_map_observation()    # 3*3
        if self.flag == 1:
            self.flag = 0
            return 

        # Odometry Model
        u_odom  = self.calc_odometry(msg)
        delta_yaw = math.atan2(u_odom[1,0], u_odom[0,0])
        s = self.xOdom
        self.xOdom[0] = s[0] + math.cos(s[2])*u_odom[0,2] - math.sin(s[2])*u_odom[1,2]
        self.xOdom[1] = s[1] + math.sin(s[2])*u_odom[0,2] + math.cos(s[2])*u_odom[1,2]
        self.xOdom[2] = s[2] + delta_yaw
        self.publishResult_odom()
        u_odom = self.T2u(u_odom)

        self.z1 = np.array(self.z1)
        self.z1 = self.z2Robot(self.z1)
        self.z2 = self.z2Robot(self.z2)
        #print "self.xEst is: ", self.xEst
        self.xEst, self.PEst = self.ekf.estimate(self.xEst, self.PEst, self.z1, self.z2, u_odom)
        #print "x is: ", self.xOdom[0], self.xEst[0][0]
        #print "y is :", self.xOdom[1], self.xEst[1][0]
        #print "z is: ", self.xOdom[2], self.xEst[2][0]
        #print "self.xEst is:  LATER", self.xEst
        self.publishResult_est()
        
    def z2Robot(self, z):
        new_z = z
        n = len(z)/2
        #print " new_z is:" ,new_z
        #print " n is: ", n
        for i in range(n):
            x = z[2*i]
            y = z[2*i+1]

            new_z[2*i] = (x-self.xEst[0]) * math.cos(self.xEst[2]) + (y-self.xEst[1]) * math.sin(self.xEst[2])
            new_z[2*i+1] = (x-self.xEst[0]) * math.sin(self.xEst[2]) + (y-self.xEst[1]) * math.cos(self.xEst[2])
            #new_z[2*i+1] = (z[2*i][0]-self.xEst[0]) * (-math.sin(self.xEst[2])) + (z[2*i+1][0]-self.xEst[1]) * math.cos(self.xEst[2])

        return new_z
        
    def laserEstimation(self,msg):     
        total_num = len(msg.ranges)
        data = deepcopy(msg)
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        angle_increment=msg.angle_increment
        data.ranges = [100.0]*total_num
        for i in range(len(self.obstacle[0])):
            dist=math.hypot((self.xEst[0]-self.obstacle[0][i]),(self.xEst[1]-self.obstacle[1][i]))
            index=int((math.atan2((self.obstacle[1][i]-self.xEst[1]),(self.obstacle[0][i]-self.xEst[0]))-msg.angle_min-self.xEst[2])/angle_increment)

            while index > total_num-1 :
                index=index-total_num
            while index < 0 :
                index=index+total_num
            if dist<data.ranges[index] :
                data.ranges[index]=dist

        self.target_laser = data
        pc_esti = self.laserToNumpy(data)
        pc_esti = pc_esti[:2]
        return pc_esti  # pc_esti: 3*100, the third row is 0 , no error

    def laserEstimation1(self,msg):  
        total_num = len(msg.ranges)
        data = deepcopy(msg)
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        angle_increment=msg.angle_increment
        data.ranges = [100.0]*total_num
        #print len(self.obstacle)
        for i in range(len(self.obstacle[0])):
            dist=math.hypot((self.xEst[0]-self.obstacle[0][i]),(self.xEst[1]-self.obstacle[1][i]))
            index=int((math.atan2((self.obstacle[1][i]-self.xEst[1]),(self.obstacle[0][i]-self.xEst[0]))-msg.angle_min-self.xEst[2])/angle_increment)

            while index > total_num-1 :
                index=index-total_num
            while index < 0 :
                index=index+total_num
            if dist<data.ranges[index] :
                data.ranges[index]=dist

        pc_esti = self.laserToNumpy(data)
        pc_esti = pc_esti[:2]
        return pc_esti  # pc_esti: 2*100

    def calc_map_observation(self):
        #tar_pc = self.laserEstimation1(msg)
        #landmarks_tar = self.extraction.process(tar_pc, True)
        landmarks_src = self.extraction.process(self.pc_laser2numpy[:2], True)
        if landmarks_src is None:
            self.flag = 1
            return 
        #print "position_x: ", landmarks_src.position_x
        #print "position_y: ", landmarks_src.position_y
        self.publishLandMark(landmarks_src)
        self.publishLandMark_g(landmarks_src)
        self.z2 = self.lm2pc(landmarks_src) # laser, 2n*1 map ordinate
        self.n = len(self.z2)/2
        self.z1 = self.icp.match(self.new_map_pc, self.z2)
        return 

    def mapToworld(self, z, resolution, x, y):
        m = z
        n = len(z)/2
        for i in range(n):
            wx = z[2*i]
            wy = z[2*i+1]
            m[2*i] = wx * resolution + x
            m[2*i+1] = wy * resolution + y
        return m

    def calc_odometry(self,msg):
        if self.isFirstScan:
            self.tar_pc = self.laserEstimation(msg)
            self.isFirstScan = False
            self.lasercount = 0
            return np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]).T
        self.src_pc = self.pc_laser2numpy   # lasertoNumpy
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = self.pc_laser2numpy
        return transform_acc

    def T2u(self, t):   # T to u, return 3*1
        # t is 3*3 transform matrix
        dw = math.atan2(t[1, 0], t[0, 0])   # dyaw
        u = np.array([[t[0, 2], t[1, 2], dw]])  # u is 1*3, [[dx, dy, dyaw]]

        return u.T  # 3*1

    def lm2pc(self, lm):
        total_num = len(lm.id)  # number of points in obstacles
        dy = lm.position_y
        dx = lm.position_x
        range_l = np.hypot(dy, dx)  # distance(y, x)
        angle_l = np.arctan2(dy, dx)
        pc = np.ones((2, total_num))
        pc[0:2, :] = np.vstack((np.multiply(np.cos(angle_l), range_l), np.multiply(np.sin(angle_l), range_l)))
        new_pc = pc.transpose().reshape(2*total_num, 1)
        #print("mdbg ", total_num)
        return new_pc   # 2n*1

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc
    
    def publishResult_est(self):
        # tf
        s = self.xEst
        #print "xEst[2] is: ", self.xEst[2]
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","world_base")
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

        self.location_pub.publish(odom)

    def publishResult_odom(self):
        s = self.xOdom
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"icp_odom","world_base")

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
        self.laser_pub.publish(self.target_laser)

    def publishLandMark(self, msg):   # msg.id[]/ msg.position_x[]/ msg.position_y[]
        if len(msg.id) <= 0:
            return
        # MarkerArray.marker
        landMark_array_msg = MarkerArray()
        for i in range(len(msg.id)):
            marker = Marker()
            marker.header.frame_id = "course_agv__hokuyo__link"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "lm"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = msg.position_x[i]
            marker.pose.position.y = msg.position_y[i]
            marker.pose.position.z = 0 # 2D
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.5 # Don't forget to set the alpha
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            landMark_array_msg.markers.append(marker)

        self.landMark_pub.publish(landMark_array_msg)

    def publishLandMark_g(self, msg):   # msg.id[]/ msg.position_x[]/ msg.position_y[]
        if len(msg.id) <= 0:
            return
        # MarkerArray.marker
        landMark_array_msg = MarkerArray()
        for i in range(len(msg.id)):
            marker = Marker()
            marker.header.frame_id = "course_agv__hokuyo__link"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "lm"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = msg.position_x[i] + random.random()
            marker.pose.position.y = msg.position_y[i] + random.random( )
            marker.pose.position.z = 0 # 2D
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.5 # Don't forget to set the alpha
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            landMark_array_msg.markers.append(marker)

        self.landMark_pub.publish(landMark_array_msg)

def main():
    rospy.init_node('localization_node')
    l = Localization()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()