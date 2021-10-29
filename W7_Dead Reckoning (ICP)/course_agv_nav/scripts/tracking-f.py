#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
import math
import time

class Tracking():
    def __init__(self):
        self.k1 = 0.3
        self.k2 = 2.5
        self.maxspeed = 2
        self.niu = 0.1
        self.lamda = 1
        self.accept = 0.2
        self.px = []
        self.py = []
        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path, self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)

    
    def pathCallback(self,msg):
        print('pathcallback')
        self.inittracking()
        self.path = msg
        for i in range(len(msg.poses)):                             #get the path and store in rx[] ry[]
            self.px.append(msg.poses[i].pose.position.x)
            self.py.append(msg.poses[i].pose.position.y)
        #    print(self.px[i],self.py[i])
        #self.goal_index = 1
        #self.getcarposition()
    #    self.starttracking()
        self.go_tracking()

    def inittracking(self):
        self.px = []
        self.py = []
        self.path = Path()
        self.goal_index = 1

    def getcarposition(self):
        #get the position of car
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]      #the position of the car
        self.y = self.trans[1]    
        self.yaw = yaw
        #print('postion of the car')
        #print(self.x,self.y,self,yaw)
        #print('goal_index')
        #print(self.goal_index)
        #print('inint_path_x=',self.px[0],'y=',self.py[0])
        #self.next_goal_x = self.px[self.goal_index]        #the position of the mid goal
        #self.next_goal_y = self.py[self.goal_index]
        #print('mid goal')
       # print(self.next_goal_x,self.next_goal_y)
        #decide whether to change the mid goal
     #   print('yaw=',self.yaw)
       
    
    def if_next(self):
        self.next_goal_x = self.px[self.goal_index]        #the position of the mid goal
        self.next_goal_y = self.py[self.goal_index]
        dis = math.hypot(self.x-self.next_goal_x,self.y-self.next_goal_y)
        if dis < self.accept and self.goal_index < len(self.path.poses)-1:
            self.goal_index = self.goal_index + 1
            return 1
        else:
            return 0

    
    def onetracking(self):
        while 1:
            self.getcarposition()
            if_ne = self.if_next()
            if if_ne == 1:
                print('go next=',self.goal_index)
                break
            self.rou = math.hypot(self.next_goal_x-self.x,self.next_goal_y-self.y)
            self.gama = math.atan2((self.next_goal_y-self.y),(self.next_goal_x-self.x))
            self.tamplapha = self.gama-self.yaw
            self.alpha = math.asin(math.sin(self.gama-self.yaw))
            self.beta = math.asin(math.sin(math.atan2((self.next_goal_y-self.py[self.goal_index-1]),(self.next_goal_x-self.px[self.goal_index-1]))-self.gama))
            self.k = -(self.k2*(self.alpha-math.atan(-self.k1*self.beta))+math.sin(self.alpha)*(1+self.k1/(1+math.pow(self.k1,2)*math.pow(self.beta,2))))/self.rou
            self.vx = self.maxspeed/(1+self.niu*(abs(self.k)**self.lamda))
            self.vw = self.vx*self.k
            if math.cos(self.tamplapha) > 0:
                self.vx = self.vx
            else:
                self.vx = -self.vx
          #  self.beta = math.atan2((self.next_goal_y-self.y),(self.next_goal_x-self.x))
           # self.alpha = -self.beta - self.yaw
            #self.k = -(self.k2*(self.alpha-math.atan(-self.k1*self.beta))+math.sin(self.alpha)*(1+self.k1/(1+math.pow(self.k1*self.beta,2))))/self.rou
            #self.vx = self.maxspeed/(1+self.niu*math.pow(abs(self.k),self.lamda))
            #self.vw = self.k*self.vx 
         #   print('vx=',self.vx)
           # if (self.alpha > -3.14 and self.alpha < -1.57) or (self.alpha > 1.57 and self.alpha < 3,14):
            #    self.vx = -self.vx 
            cmd = Twist()
            cmd.linear.x = self.vx
            cmd.angular.z = self.vw
           # print('v=',self.vx,'w=',self.vw)
            self.vel_pub.publish(cmd)
            rospy.sleep(0.01)
        
    def go_tracking(self):
        print('tracking')
        while self.goal_index < len(self.path.poses)-1:
            self.onetracking()
        cmd = Twist()
        if self.goal_index == len(self.path.poses)-1:
            print('Arrived!!!')
            self.vw = 0
            while self.vx > 0.05:
                self.vx = self.vx - 0.05
                cmd.linear.x = self.vx
                cmd.angular.z = self.vw
                self.vel_pub.publish(cmd)
                rospy.sleep(0.01)
            self.vx = 0
            cmd.linear.x = self.vx
            self.vel_pub.publish(cmd)
            rospy.sleep(0.1)





def main():
    rospy.init_node('tracking')
    Tracking()
    rospy.spin()

if __name__=='__main__':
    main()
