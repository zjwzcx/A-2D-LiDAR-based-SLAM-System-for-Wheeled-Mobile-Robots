#!/usr/bin/env python
import rospy
import tf
import math
import random
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker   # Landmark
import numpy as np
from icp import ICP
from ekf import EKF
from extraction import LandMarkSet,  Extraction  # Extract landmark
from copy import deepcopy 
import sys
class Localization():
    def __init__(self):
        self.icp = ICP()
        self.ekf = EKF()
        self.extraction = Extraction()

        # odom robot init states
        self.robot_x = rospy.get_param('/icp/robot_x', 0)
        self.robot_y = rospy.get_param('/icp/robot_y', 0)
        self.robot_theta = rospy.get_param('/icp/robot_theta', 0)
        self.sensor_sta = [self.robot_x, self.robot_y, self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []

        # State Vector [x y yaw]
        self.xOdom = np.zeros((3, 1))
        self.xEst = np.zeros((3, 1))
        self.PEst = np.eye(3)
        self.laser_count = 0
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # init map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.updateMap)
        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan', LaserScan, self.laserCallback)
        self.location_pub = rospy.Publisher('ekf_location', Odometry, queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom', Odometry, queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.landMark_pub = rospy.Publisher('/landMarks', MarkerArray, queue_size=1)
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)

    def updateMap(self,  msg):
        print('start getting map')
        self.map = msg
        # Update for planning algorithm
        map_data = np.array(self.map.data).reshape((-1, self.map.info.height)).transpose()
        tx, ty = np.nonzero((map_data > 20)|(map_data < -0.5))
        ox = (tx*self.map.info.resolution+self.map.info.origin.position.x)*1.0
        oy = (ty*self.map.info.resolution+self.map.info.origin.position.y)*1.0
        self.obstacle = np.vstack((ox, oy))   #transpose()
        self.obstacle_r = self.map.info.resolution
        print("debug: update map obstacle success! ")


    def laserCallback(self, msg):

        self.laser_count += 1
        if self.laser_count <= 5:
            return
        self.laser_count = 0
        laser_pc = self.laserToNumpy(msg)   # point_cloud
        u = self.calc_map_observation(laser_pc)
        '''
        u_odom  = self.calc_odometry(msg)
        delta_yaw = math.atan2(u_odom[1,0], u_odom[0,0])
        s = self.xOdom
        print "u_odom is:  " , u_odom
        self.xOdom[0] = s[0] + math.cos(s[2])*u_odom[0,2] - math.sin(s[2])*u_odom[1,2]
        self.xOdom[1] = s[1] + math.sin(s[2])*u_odom[0,2] + math.cos(s[2])*u_odom[1,2]
        self.xOdom[2] = s[2] + delta_yaw
        self.publishResult()
        '''


    def laserEstimation(self, msg, x):
        # TODO
        total_num = len(msg.ranges)
        data = deepcopy(msg)
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        angle_increment=msg.angle_increment
        data.ranges = [100.0]*total_num
        #print len(self.obstacle)
        for i in range(len(self.obstacle[0])):
            dist=math.hypot((self.xEst[0]-self.obstacle[0][i]),(self.xEst[1]-self.obstacle[1][i]))
            index=int((math.atan2((self.obstacle[1][i]-self.xEst[1]),(self.obstacle[0][i]-self.xEst[0]))-msg.angle_min-x[2])/angle_increment)
            #print index
            while index > total_num-1 :
                index=index-total_num
            while index < 0 :
                index=index+total_num
            if dist<data.ranges[index] :
                data.ranges[index]=dist
        self.target_laser = data

        pc_esti = self.laserToNumpy(data)
        print "pc_esti", pc_esti
        return pc_esti  # pc_esti: 3*100, the third row is 0 , no error

    def calc_map_observation(self, msg):    # msg is self.tar_pc, that is pc_array
        time_1 = rospy.get_time()
        landmarks_src = self.extraction.process(msg, True)
        if landmarks_src is None:
            return 
        time_2 = rospy.get_time()
        print "delta time is: ", time_2-time_1
        #print "position_x is: ", landmarks_src.position_x
        #print "position_y is: ", landmarks_src.position_y
        self.publishLandMark(landmarks_src)

        return 

    def calc_odometry(self, msg):
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(msg)
            self.isFirstScan = False
            return np.array([[0.0, 0.0, 0.0]]).T
        self.src_pc = self.laserToNumpy(msg)
        transform_acc = self.icp.process(self.tar_pc, self.src_pc)
        self.tar_pc = self.laserToNumpy(msg)    # update the latest point_cloud
        # return self.T2u(transform_acc) #yuanlaide
        return transform_acc

    def lm2pc(self, lm):
        total_num = len(lm.id)  # number of points in obstacles
        dy = lm.position_y
        dx = lm.position_x
        range_l = np.hypot(dy, dx)
        angle_l = np.arctan2(dy, dx)
        pc = np.ones((3, total_num))
        pc[0:2, :] = np.vstack((np.multiply(np.cos(angle_l), range_l), np.multiply(np.sin(angle_l), range_l)))
        print("mdbg ", total_num)
        return pc

    def laserToNumpy(self, msg):    # msg is LaserScan
        total_num = len(msg.ranges)
        pc = np.ones([3, total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min, msg.angle_max, total_num)
        pc[0:2, :] = np.vstack((np.multiply(np.cos(angle_l), range_l), np.multiply(np.sin(angle_l), range_l)))
        return pc
        
    def T2u(self, t):   # T to u
        # t is 3*3 transform matrix

        dw = math.atan2(t[1, 0], t[0, 0])   # dyaw
        u = np.array([[t[0, 2], t[1, 2], dw]])  # u is 1*3, [[dx, dy, dyaw]]
        return u.T  # 3*1

    def publishResult(self):
        '''
        # tf
        s = self.xEst.reshape(-1)
        q = tf.transformations.quaternion_from_euler(0, 0, s[2])
        self.odom_broadcaster.sendTransform((s[0], s[1], 0.001), (q[0], q[1], q[2], q[3]), 
                            rospy.Time.now(), "ekf_location", "world_base")
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
        '''
        s = self.xOdom
        q = tf.transformations.quaternion_from_euler(0, 0, s[2])
        self.odom_broadcaster.sendTransform((s[0], s[1], 0.001), (q[0], q[1], q[2], q[3]), 
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

        #self.laser_pub.publish(self.target_laser)

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

            #print(marker.pose)
        #if len(msg.id) <= 0:
            #return
        # MarkerArray.marker
        #landMark_array_msg = MarkerArray()
        for i in range(len(msg.id)):
            marker = Marker()
            marker.header.frame_id = "course_agv__hokuyo__link"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "lm"
            marker.id = i + len(msg.id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = msg.position_x[i] + random.random()/8
            marker.pose.position.y = msg.position_y[i] + random.random()/8
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

if __name__ == '__main__':
    main()
