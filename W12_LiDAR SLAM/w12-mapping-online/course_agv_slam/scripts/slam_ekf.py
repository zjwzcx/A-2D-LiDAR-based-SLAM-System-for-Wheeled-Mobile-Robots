#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray,Marker
from nav_msgs.msg import OccupancyGrid
import numpy as np
from icp import ICP
from ekf_lm import EKF,STATE_SIZE
import ekf_lm as ekf
from extraction import LandMarkSet,Extraction
from mapping import Mapping
import sys

MAX_LASER_RANGE = 30

class SLAM_EKF():
    def __init__(self):
        # ros param
        self.robot_x = rospy.get_param('/slam/robot_x',0)
        self.robot_y = rospy.get_param('/slam/robot_y',0)
        self.robot_theta = rospy.get_param('/slam/robot_theta',0)

        # ros param of mapping
        self.map_x_width = rospy.get_param('/slam/map_width')
        self.map_y_width = rospy.get_param('/slam/map_height')
        self.map_reso = rospy.get_param('/slam/map_resolution')
        self.map_cellx_width = int(round(self.map_x_width/self.map_reso))
        self.map_celly_width = int(round(self.map_y_width/self.map_reso))
        self.mapping = Mapping(self.map_cellx_width, self.map_celly_width, self.map_reso)

        self.icp = ICP()
        self.ekf = EKF()
        self.extraction = Extraction()
        
        # odom robot init states
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []
        self.laser_count = 0
        # State Vector [x y yaw]
        self.xOdom = np.zeros((STATE_SIZE, 1))
        self.xEst = np.zeros((STATE_SIZE, 1))
        self.PEst = np.eye(STATE_SIZE)
        self.x_online = 0
        self.y_online = 0
        self.z_online = 0
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # ros topic
        #self.pose_pub = rospy.Publisher('/pose')
        #self.pose_sub = rospy.Subscriber('/pose', PoseStamped)
        #self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size = 10)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.landMark_pub = rospy.Publisher('/landMarks',MarkerArray,queue_size=1)
        self.map_pub = rospy.Publisher('/slam_map',OccupancyGrid,queue_size=1)

    def tf_callback(self, msg):
        for i in msg.transforms:
            self.x_online = i.transform.translation.x
            self.y_online = i.transform.translation.y
            self.z_online = i.transform.translation.z
        pass

    def laserCallback(self, msg):
        # valid frames
        self.laser_count += 1
        if self.laser_count <= 5:
            return
        self.laser_count = 0
        
        np_msg = self.laserToNumpy(msg) # 3*laser_num(100/300)
        if self.isFirstScan:
            self.isFirstScan = False
            self.tar_pc = np_msg
            self.laser_count = 0
            return
        lm = self.extraction.process(np_msg)
        if lm is None:
            print "NoneType" 
            return
        
        u = self.calc_odometry(np_msg)  # 3*1
        z = self.observation(lm)    # n*3
        self.xEst, self.PEst = self.ekf.estimate(self.xEst,self.PEst,z,u)
        
        # Map Building
        obs = self.u2T(self.xEst[:3]).dot(np_msg)   # (2*3) * (3*laser_num)


        pmap = self.mapping.update(obs[0], obs[1], self.x_online, self.y_online)
        self.publishMap(pmap)
        
        self.publishLandMark(lm)
        self.publishResult()
        return

    def observation(self, lm):
        landmarks = lm
        z = np.zeros((0, 3))
        for i in range(len(landmarks.id)):
            dx = landmarks.position_x[i]
            dy = landmarks.position_y[i]
            d = math.hypot(dx, dy)
            angle = self.ekf.pi_2_pi(math.atan2(dy, dx))
            zi = np.array([d, angle, i])
            z = np.vstack((z, zi))
        return z    # n*3, now there are n landmarks

    def calc_odometry(self,np_msg):
        self.src_pc = np_msg
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = np_msg
        return self.T2u(transform_acc)  # 3*1

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        range_l[range_l == np.inf] = MAX_LASER_RANGE
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc   # 3*laser_num

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        return u.T  # 3*1
    
    def u2T(self,u):    # return 2*3
        dx = u[0]
        dy = u[1]
        w = u[2]
        return np.array([
            [math.cos(w),-math.sin(w), dx],
            [math.sin(w), math.cos(w), dy]
        ])

    def lm2pc(self,lm):
        total_num = len(lm.id)
        dy = lm.position_y
        dx = lm.position_x
        range_l = np.hypot(dy,dx)
        angle_l = np.arctan2(dy,dx)
        pc = np.ones((3,total_num))
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        print("mdbg ",total_num)
        return pc

    def publishResult(self):
        # tf
        s = self.xEst.reshape(-1)
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
        
        #print("mdbg ",self.xEst.shape)
        # self.laser_pub.publish(self.target_laser)
        pass

    def publishLandMark(self, msg):
        '''
        blue ball is the result of extraction.process()
        green ball is the position of landmarks in self.xEst
        '''
        # msg = LandMarkSet()
        if len(msg.id) <= 0:
            return
        
        landMark_array_msg = MarkerArray()
        '''
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
        '''
        for i in range(((self.xEst.shape[0]-STATE_SIZE)/2)):
            marker = Marker()
            marker.header.frame_id = "world_base"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "lm"
            marker.id = i+len(msg.id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.xEst[STATE_SIZE+i*2,0]
            marker.pose.position.y = self.xEst[STATE_SIZE+i*2+1,0]
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

    def publishMap(self,pmap):
        map_msg = OccupancyGrid()
        map_msg.header.seq = 1
        map_msg.header.stamp = rospy.Time().now()
        map_msg.header.frame_id = "map"

        map_msg.info.map_load_time = rospy.Time().now()
        map_msg.info.resolution = self.map_reso
        map_msg.info.width = self.map_cellx_width
        map_msg.info.height = self.map_celly_width
        map_msg.info.origin.position.x = -self.map_cellx_width*self.map_reso/2.0
        map_msg.info.origin.position.y = -self.map_celly_width*self.map_reso/2.0
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = list(pmap.T.reshape(-1))
        map_msg.data  = np.trunc(map_msg.data).astype(np.int8).tolist()
        print "map: ", len(map_msg.data)
        #print map_msg.data
        
        self.map_pub.publish(map_msg)

def main():
    rospy.init_node('slam_node')
    s = SLAM_EKF()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
