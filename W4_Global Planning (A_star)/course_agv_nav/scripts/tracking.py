#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from threading import Lock,Thread
import math
import time
class Tracking: # Subscriber -> pathCallback() -> initTracking()/ -> updateGlobalPose() -> trackThreadFunc() -> planOnce()
    def __init__(self):
        self.arrive_threshold = 0.2
        self.vx = 0.0
        self.vw = 0.0
        self.lock = Lock()
        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path', Path, self.pathCallback)    # receive the initial path and start tracking
        self.vel_pub = rospy.Publisher('/course_agv/velocity', Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal', PoseStamped, queue_size=1)   # updateGlobalPose
        self.tracking_thread = None
        pass

    def pathCallback(self,msg):
        self.path = msg
        self.lock.acquire()
        self.initTracking()
        self.lock.release() # start?
        if self.tracking_thread == None:
            self.tracking_thread = Thread(target = self.trackThreadFunc)
            self.tracking_thread.start()
        pass

    def initTracking(self):
        self.goal_index = 0
        self.updateGlobalPose()
        pass

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))  # transform map into robot_base
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)  # automatically compute the euler_angle
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw  # theta

        p = self.path.poses[self.goal_index].pose.position
        dis = math.hypot(p.x-self.x,p.y-self.y)
        if dis < self.arrive_threshold and self.goal_index < len(self.path.poses)-1:    # hasn't arrived yet
            self.goal_index = self.goal_index + 1   # steps
        self.midpose_pub.publish(self.path.poses[self.goal_index])  # publish the midpose

    def trackThreadFunc(self):  # import velocity
        print("start planOnce!!")
        while True:
            self.planOnce()
            time.sleep(0.1)
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(True)
        self.lock.release()
        self.tracking_thread = None
        pass

    def planOnce(self): # compute the velocity
        self.lock.acquire()
        self.updateGlobalPose()
        target = self.path.poses[self.goal_index].pose.position # next target 
        dx = target.x - self.x  # delta x
        dy = target.y - self.y  # delta y
        rho = math.sqrt(dx**2 + dy**2)

        beta = math.atan2(dy, dx)   # beta in (0, pi), theta in (-pi, pi)
        #alpha = pass
        
        alpha = beta - self.yaw
        #alpha = math.asin(math.sin(alpha))
        '''
        if alpha < -math.pi:    # alpha in (-pi, pi)
            alpha += 2 * math.pi
        elif alpha >= math.pi:
            alpha -= 2 * math.pi
        '''
        print "The step is: ", self.goal_index
        print "alpha is: ", alpha
        print "beta is: ", beta
        print "theta is: ", self.yaw
        #k1, k2, k3 = 0.88, 3, 1 # k3 determine the time robot goes straight
        k1, k2 = 0.88, 3

        gamma = k2 * (alpha - math.atan(-k1 * beta)) + (1 + k1/(1 + (k1*beta)**2))*math.sin(alpha)
        k = gamma/rho  # curvature
        miu, langda, v_max = 0.2, 1, 5

        self.vx = v_max/(1 + miu * abs(k)**langda)
        self.vw = k * self.vx
        # self.vw = -k3 * gamma
        '''
        if alpha < -math.pi/2:        
            self.vw = 20
        elif alpha > math.pi/2:
            self.vw = -20
        self.vw = 50
        '''
        print "k is: ", k
        print "vx is: ", self.vx, "vw is: ", self.vw
        print "==============="
        self.publishVel()
        self.lock.release()
        pass

    def publishVel(self,zero = False): # publish cmd message to kinematics.py
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.vel_pub.publish(cmd)   # publish data.linear.x & data.angular.z

def main():
    rospy.init_node('tracking')
    t = Tracking()
    rospy.spin()

if __name__ == '__main__':
    main()
