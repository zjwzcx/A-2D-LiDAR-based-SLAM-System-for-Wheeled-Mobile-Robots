#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
def controller(data):
	publ = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command', Float64,queue_size=10)
	pubr = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command', Float64,queue_size=10)
	wr = Float64((1.0*data.linear.x + 0.2*data.angular.z)/2)
	wl = Float64((1.0*data.linear.x - 0.2*data.angular.z)/2)
	
	rospy.loginfo(wl)
	rospy.loginfo(wr)
	publ.publish(wl)
	pubr.publish(wr)
def listener():
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber('/course_agv/velocity', Twist, controller)
	rospy.spin()
if __name__ == '__main__':
	listener()
