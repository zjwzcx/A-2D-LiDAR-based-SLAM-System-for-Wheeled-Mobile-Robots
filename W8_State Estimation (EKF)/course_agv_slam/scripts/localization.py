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
class Localization():
    def __init__(self):
        self.icp = ICP()
        self.ekf = EKF()    # call self.ekf.estimate
        
        # odom robot init states
        self.robot_x = rospy.get_param('/icp/robot_x', 0)
        self.robot_y = rospy.get_param('/icp/robot_y', 0)
        self.robot_theta = rospy.get_param('/icp/robot_theta', 0)
        self.sensor_sta = [self.robot_x, self.robot_y, self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []
        
        self.resolution = 0
        self.origin_x = 0
        self.origin_y = 0
        self.width = 0
        self.height = 0
        self.tx = 0
        self.ty = 0

        # State Vector [x y yaw]
        #self.xOdom = [[0], [0], [0], [0]]
        #self.xEst = [[0], [0], [0], [0]]
        self.xOdom = [0, 0, 0]
        self.xEst = [0, 0, 0]
        self.PEst = [[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]]
        
        # map observation
        #self.obstacle = []
        self.ox = 0
        self.oy = 0
        # radius
        self.obstacle_r = 10

        self.laser_count = 0
        self.target_laser = LaserScan()

        # init map
        self.updateMap()
        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan', LaserScan, self.laserCallback)
        self.laser_pub = rospy.Publisher('target_laser', LaserScan, queue_size=3)
        self.location_pub = rospy.Publisher('ekf_location', Odometry, queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom', Odometry, queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.laser_broadcaster = tf.TransformBroadcaster()


    def updateMap(self):
        print("debug: try update map obstacle")
        try:
            self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        ox = (self.tx * self.resolution + self.origin_x) * 1.0
        oy = (self.ty * self.resolution + self.origin_y) * 1.0
        #self.obstacle = np.vstack((ox, oy)).transpose() # 1*2
        #print "ox is:", ox, "oy is: ", oy
        #print self.obstacle
        self.obstacle_r = self.resolution
        print("debug: update map obstacle success! ")

    def map_callback(self, msg):
        raw = np.array(msg.data, dtype = 'int8')
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        map_data = np.array(msg.data).reshape((-1, self.height)).transpose()
        self.tx, self.ty = np.nonzero((map_data > 20)|(map_data < -0.5))

    def laserCallback(self, msg):
        if self.laser_count < 5:
            self.laser_count += 1
            return
        self.laser_count = 0
        
        T_odom = self.calc_odometry(msg)    # ICP in w7, msg is laser in t
        u = self.trans(self.xOdom, T_odom)
        u[0] -= self.xOdom[0]
        u[1] -= self.xOdom[1]
        u[2] -= self.xOdom[2]
        self.xOdom = self.trans(self.xOdom, T_odom) # 

        xPre = self.trans(self.xEst, T_odom)
        T_observation = self.calc_map_observation(msg)
        self.xEst = self.trans(xPre, T_observation)
        '''
        1 is the estimating state vector of x_{t-1}, 4*1
        2 is covariace matrix of the state, 4*4
        3 is the observation of absolute postion（绝对观测的间接测量）
        4 is delta_x
        '''
        self.xEst, self.PEst = self.ekf.estimate(self.xOdom, self.PEst, self.xEst, u)  # xOdom is 
        self.publishResult()

    def calc_odometry(self, msg):
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(self.laserEstimation(msg, self.xEst))
            # print("ddddddddddddddddddd ",self.tar_pc - self.laserToNumpy(msg))
            self.isFirstScan = False
        self.src_pc = self.laserToNumpy(msg)
        transform_acc = self.icp.process(self.tar_pc, self.src_pc)
        self.tar_pc = self.laserToNumpy(msg)
        return transform_acc
    
    def laserEstimation(self, msg, x):  # initial value of laser
        x = x[:3]
        esti_msg = msg
        #print "x is: ", x
        dx = self.ox - x[0]
        dy = self.oy - x[1]
        alpha = math.atan2(dy, dx) - x[2]

        rou = math.hypot(dx, dy)    # max_distance
        delta_alpha = math.atan2(self.obstacle_r/2, rou)
        alpha_min = math.atan2(math.sin(alpha-delta_alpha), math.cos(alpha-delta_alpha))
        alpha_max = math.atan2(math.sin(alpha+delta_alpha), math.cos(alpha+delta_alpha))
        index_min = int(np.floor(abs((alpha_min - msg.angle_min)/msg.angle_increment)))
        index_max = int(np.floor(abs((alpha_max - msg.angle_min)/msg.angle_increment)))
        
        list_temp = list(esti_msg.ranges)
        if index_min <= index_max:
            for index in range(index_min, index_max+1):
                if list_temp[index] > rou:
                    list_temp[index] = rou
        else:
            for index in range(0, index_max+1):
                if list_temp[index] > rou:
                    list_temp[index] = rou
            for index in range(index_min, len(msg.ranges)):
                if list_temp[index] > rou:
                    list_temp[index] = rou
        esti_msg.ranges = tuple(list_temp)
        self.target_laser = esti_msg

        return esti_msg

    def calc_map_observation(self, msg):  # observation model
        # msg is observation result of laser in time t.
        # xPre is used to simulate a point cloud distribution
        # # msg is initial value of laser, map_ICP --> EKF

        tar_pc = self.laserToNumpy(self.laserEstimation(msg, self.xEst))    # Estimation
        transform_acc = self.icp.process(tar_pc, self.src_pc)   # ICP

        return transform_acc

    def trans(self, x_state, T):  # x_state is 4*1 column vector, T is 3*3 matrix
        #print x_state
        x_state = x_state[:3] # 1*3

        delta_yaw = math.atan2(T[1,0],T[0,0])
        #print "x_state[2] is: ", x_state[2]
        s = x_state
        x_state[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        x_state[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        x_state[2] = s[2] + delta_yaw
        #print "sensor-global: ",self.sensor_sta

        x_next = [0, 0, 0]
        x_next[0] = x_state[0]
        x_next[1] = x_state[1]
        x_next[2] = x_state[2]

        return x_next

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def publishResult(self):
        # Odometry Model: Red ====== W7 ICP
        s = self.xOdom  ################
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.laser_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"target_laser", "world_base")

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.5
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)


        # Observation Model: Blue ======= W8 EKF
        s = self.xEst   ################
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","world_base")

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


        self.laser_pub.publish(self.target_laser)

def main():
    rospy.init_node('localization_node')
    l = Localization()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
