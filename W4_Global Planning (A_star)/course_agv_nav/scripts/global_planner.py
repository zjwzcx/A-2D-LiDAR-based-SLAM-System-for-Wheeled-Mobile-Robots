#!/usr/bin/env python
import rospy
import tf
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from course_agv_nav.srv import Plan,PlanResponse
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int8
import time

class GlobalPlanner:
    def __init__(self):
        self.path_pub = rospy.Publisher("/course_agv/global_path", Path, queue_size=1)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.origin_x = 0
        self.origin_y = 0
        self.resolution = 0
        self.width = 0
        self.height = 0
        self.current_path = Path()
        rospy.sleep(1.0)

        # init node and goal node
        self.start_map_point = []
        self.goal_map_point = []
        
        self.path_map = []
        self.path_world = []
        
        self.if_start_find_path = False
        self.init_pose = Pose() # pose and twist
        self.goal_pose = PoseStamped()  # pose
        self.init_pose_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.init_pose_callback) # pose/twist in world frame
        self.goal_pose_sub = rospy.Subscriber("/course_agv/goal", PoseStamped, self.goal_pose_callback) # put msg as arguments of callback
        self.last_time = rospy.get_rostime()
        self.start_find_path()
        rospy.Rate(0.5)
        rospy.spin()

    def init_pose_callback(self, msg):
        self.init_pose = msg
        self.start_map_point =  self.WorldTomap(msg.pose[1].position.x, msg.pose[1].position.y)
        if self.start_map_point == [-1, -1]:
            print "error init"  # -1 is oracle in map

    def goal_pose_callback(self, msg):  # input msg as the goal point 
        self.path_map = []
        self.goal_pose = msg
        self.if_start_find_path = True
        self.goal_map_point =  self.WorldTomap(msg.pose.position.x, msg.pose.position.y)
        print "goal point is: ",self.goal_map_point
        if self.goal_map_point == [-1, -1]:
            print "error goal"
            return
        else:
            self.start_find_path()
    
    def map_callback(self, msg):
        #print "map information is: "msg.info
        #print "------"
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        raw = np.array(msg.data, dtype = Int8)
        self.map = raw.reshape((self.height, self.width))
        
    def WorldTomap(self, wx, wy):
        if wx < self.origin_x or wy < self.origin_y:
            return [-1, -1]
        mx = (int)((wx - self.origin_x) / self.resolution)
        my = (int)((wy - self.origin_y) / self.resolution)
        if mx < self.width and my < self.height:
            return [my, mx]
        return [-1, -1]

    def mapToWorld(self, wy, wx):
        mx = (wx * self.resolution + self.origin_x+0.25 )
        my = (wy * self.resolution + self.origin_y+0.25 )
        return [my, mx]
        

    def start_find_path(self):
        if self.if_start_find_path:
            temp = find_path(self.map, self.start_map_point, self.goal_map_point)
            self.path_map = temp.start_find()   # find the path
            self.path_map.reverse()
            print self.path_map
            self.publisher_path()
        else:
            print('test the sleep time')
            rospy.sleep(0.1)
            print ('waiting for goal point...')
            return 

    def publisher_path(self):
        time = 1
        y1 = []
        y2 = []
        for i in range(len(self.path_map)):
            current_time = rospy.get_rostime()
            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y= self.mapToWorld(self.path_map[i][1], self.path_map[i][0])
            y1.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0])[0])
            y2.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0])[1])
            current_pose.pose.position.z = 0.01
            current_pose.pose.orientation.x = 0.0
            current_pose.pose.orientation.y = 0.0
            current_pose.pose.orientation.z = 0.0
            current_pose.pose.orientation.w = 1.0
            time += 1
            self.current_path.header.stamp = current_time
            self.current_path.header.frame_id = "map"
            self.current_path.poses.append(current_pose)
        #print(self.current_path)
        self.path_pub.publish(self.current_path)
        self.last_time = current_time

class map_node():
    def __init__(self):
        self.x = 0
        self.y = 0
        # F = G + H
        self.cost_f = 0
        self.cost_g = 0
        self.cost_h = 0
        self.parent = [0,0]

class find_path():
    def __init__(self, map, start, goal):
        self.map = map
        self.state_map = np.zeros([len(map[0]) , len(map[0])])
        self.start = start  # start point
        self.start[0] -= 1
        self.start[1] -= 1
        self.goal = goal    # goal point
        self.goal[0] -= 1
        self.goal[1] -= 1
        self.open_list = []
        self.close_list = []
        self.path = []
        self.if_reach = False
    
    def start_find(self):
        r = 2
        for i in range(r, 129 - r):
            for j in range(r, 129 - r):
                if self.map[i][j] == 100 or self.map[i][j] == -1:
                    for k in range(-r, r+1):
                        for t in range(-r, r+1):
                            self.map[i+k][j+t] = 99
        print "start point is: ",self.start
        if self.map[self.start[0]][self.start[1]] != 0:
            print "invalid start point: ", self.map[self.start[0]][self.start[1]]
            return "None"
        if self.map[self.goal[0]][self.goal[1]] != 0:
            print "invalid goal point."
            return "None"
        self.append_around_open(self.start, cost_g = 0)

        temp = map_node()
        temp.x = self.start[0]
        temp.y = self.start[1]
        print "temp is: ", temp.x, temp.y
        self.append_close(temp)
        while True:
            min_cost, index_min_cost = self.find_min_cost_f()
            current_node = self.open_list[index_min_cost]
            if current_node.x == self.goal[0] and current_node.y == self.goal[1]:
                self.append_path(current_node)
                break
            self.append_around_open([current_node.x, current_node.y], cost_g = current_node.cost_g)
            self.append_close(current_node) # append into close list
            self.open_list.remove(current_node)
        return self.path
    
    def append_around_open(self, coordinate, cost_g):   # find next point in 3*3 around region
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if self.map[coordinate[0] + i][coordinate[1] + j] == 0 \
                        and self.state_map[coordinate[0] + i][coordinate[1] + j] != 3:
                    temp = map_node()
                    
                    temp.cost_g = 10 + cost_g
                    temp.cost_h = (abs(self.goal[0] - (coordinate[0] + i)) + abs(self.goal[1] - (coordinate[1] + j))) * 10
                    temp.cost_f = temp.cost_g + temp.cost_h
                    temp.x = coordinate[0] + i
                    temp.y = coordinate[1] + j
                    
                    temp.parent[0] = coordinate[0]
                    temp.parent[1] = coordinate[1]
                    if self.state_map[coordinate[0] + i][coordinate[1] + j] == 2:
                        current_index = self.find_index(coordinate[0] + i, coordinate[1] + j)
                        if self.open_list[current_index].cost_f > temp.cost_f:
                            self.open_list[current_index] = temp
                    else:
                        self.state_map[coordinate[0] + i][coordinate[1] + j] = 2
                        self.open_list.append(temp)

    def append_path(self, node):
        while True:
            #self.path.append([node.x - 1, node.y - 1])
            self.path.append([node.x, node.y])
            if node.x == self.start[0] and node.y == self.start[1]:
                break
            current_index = self.find_close_index(node.parent[0], node.parent[1])
            node = self.close_list[current_index]

        
    def find_min_cost_f(self):
        min_cost = 100000
        index_min_cost = 0
        for i in range(len(self.open_list)):
            if self.open_list[i].cost_f < min_cost:
                min_cost = self.open_list[i].cost_f
                index_min_cost = i
        return min_cost, index_min_cost

    def find_close_index(self, x, y):
        for i in range(len(self.close_list)):
            if self.close_list[i].x == x and self.close_list[i].y == y:
                return i

    def find_index(self, x, y):
        for i in range(len(self.open_list)):
            if self.open_list[i].x == x and self.open_list[i].y == y:
                return i
  
    def append_close(self, node):
        self.state_map[node.x][node.y] = 3
        self.close_list.append(node)


def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    gp.map_callback(gp.map_sub)
    gp.init_pose_callback(gp.init_pose_sub)
    gp.goal_pose_callback(gp.goal_pose_sub)
    time.sleep(0.5)
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
