#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
from icp import ICP
from ekf import EKF
import sys
from copy import deepcopy 

class Localization():
    def __init__(self):
        self.icp = ICP()
        self.ekf = EKF()
        self.laser_count = 0
        
        # odom robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []

        # State Vector [x y yaw]
        self.xOdom = [0,0,0]
        self.xEst = [0,0,0]
        self.PEst = np.eye(3)
        self.test = np.zeros(3).T
        
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10
        self.new = [0,0,0]
        self.new2 = [0,0,0]

        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.updateMap)
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()


    def updateMap(self,msg):
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
        

    def laserCallback(self,msg):
        if self.laser_count == 0:
            self.time_start = rospy.get_time()
            print('start time',self.time_start)
        self.laser_count += 1
        if self.laser_count <= 5:
            return
        self.laser_count = 0
        
        # the motion odomerty model
        T = self.calc_odometry(msg)
        #print T
        delta_yaw = math.atan2(T[1,0],T[0,0])
        s = self.xOdom
        self.xOdom[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        self.xOdom[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        self.xOdom[2] = s[2] + delta_yaw

        #test ekf
        
        #print('test',self.test)
        #print('Pest',self.PEst)
        '''
        print(self.test[0])
        print(self.test[1])
        print(self.test[2])
        '''

        # the absoluate position and odomerty
        # the absoluate position
        T=self.calc_odometry(msg)
        #print T
        delta_yaw = math.atan2(T[1,0],T[0,0])
        s = self.xEst
        self.new[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        self.new[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        self.new[2] = s[2] + delta_yaw


        
        t=self.calc_map_observation(msg)
        delta_yaw = math.atan2(t[1,0],t[0,0])
        s = self.xEst
        # z is the self.new
        self.new2[0] = s[0] + math.cos(s[2])*t[0,2] - math.sin(s[2])*t[1,2]
        self.new2[1] = s[1] + math.sin(s[2])*t[0,2] + math.cos(s[2])*t[1,2]
        self.new2[2] = s[2] + delta_yaw

        self.xEst, self.PEst = self.ekf.estimate(self.xEst,self.PEst,self.new2,T)
        self.publishResult()
        pass
    
    def laserEstimation(self,msg,x):  
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
        self.target_laser=data
        #self.laser_pub.publish(self.target_laser)
        #print self.target_laser
        return data

    def calc_map_observation(self,msg):
        # TODO
        transform_acc = np.identity(3)
        tar_pc = self.laserToNumpy(self.laserEstimation(msg,self.xEst))
        transform_acc = self.icp.process(tar_pc,self.src_pc)
        return transform_acc

    def calc_odometry(self,msg):
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(self.laserEstimation(msg,self.xEst))
            # print("ddddddddddddddddddd ",self.tar_pc - self.laserToNumpy(msg))
            self.isFirstScan = False
            self.laser_count = 0
        self.src_pc = self.laserToNumpy(msg)
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = self.laserToNumpy(msg)
        return transform_acc

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc
    def publishResult(self):
        # tf
        s = self.xEst
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
        pass
def main():
    rospy.init_node('localization_node')
    l = Localization()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
