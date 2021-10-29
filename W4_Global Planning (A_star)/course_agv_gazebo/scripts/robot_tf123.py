#!/usr/bin/env python
import pandas as pd
import os
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64

dataSheet = pd.DataFrame(columns=['x','y','z'])

def callback(data):

    for i,x in enumerate(data.name):
        if x =='course_agv':
            # print(x)
            pose = data.pose[i]
            ori = pose.orientation
            pos = pose.position
            time = rospy.get_time()
            while time == 0:
                time = rospy.get_time()
            #dataSheet.loc[time]=[pos.x,pos.y,pos.z]
            #print time
            break
    pass

rospy.init_node('get_gazebo',anonymous=True)
rospy.Subscriber('/gazebo/model_states',ModelStates,callback)
rospy.spin()
path = "/home/course-ubuntu/Desktop/c3_return_data"
if not os.path.exists(path):
    os.makedirs(path)
dataSheet.to_csv(path+'/time_'+str(rospy.get_time())+'_data.csv')

print 'Data_recorded'