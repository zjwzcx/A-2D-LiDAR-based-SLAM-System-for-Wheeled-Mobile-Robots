#!/usr/bin/env python
import rospy
import tf

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

def handle_robot_pose(msg):
    for i, x in enumerate(msg.name):
        if x == 'course_agv':
            pos = msg.pose[i].position
            ori = msg.pose[i].orientation
            br = tf.TransformBroadcaster()
            br.sendTransform((pos.x, pos.y, pos.z),
                            (ori.x, ori.y, ori.z, ori.w),
                            rospy.Time.now(),
                            'robot_base',
                            'map')
            break


if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster', anonymous = True)
    rospy.Subscriber('/gazebo/model_states',
                    ModelStates,
                    handle_robot_pose)
    rospy.spin()