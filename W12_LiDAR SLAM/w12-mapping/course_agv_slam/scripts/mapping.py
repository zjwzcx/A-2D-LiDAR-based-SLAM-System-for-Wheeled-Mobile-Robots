import math
import numpy as np
import rospy
import bresenham as drawing
from nav_msgs.msg import OccupancyGrid

class Mapping():
    def __init__(self, xw, yw, xyreso):
        self.width_x = xw * xyreso  # 20
        self.width_y = yw * xyreso  # 20
        self.xyreso = xyreso    # 0.1
        self.xw = xw    # 200
        self.yw = yw    # 200
        self.pmap = 50 * np.ones((self.xw, self.yw)) # default 50, unknown pixels
        self.datamap = np.zeros((self.xw, self.yw))
        self.minx = -self.width_x/2.0
        self.maxx =  self.width_x/2.0
        self.miny = -self.width_y/2.0
        self.maxy =  self.width_y/2.0
        pass

    def update(self, ox, oy, center_x, center_y):
        '''
        center_x: xEst[0]
        center_y: xEst[1]
        '''

        n = len(ox) # number of laser
        for i in range(n):
            if np.isinf(ox[i]): 
                pass
            else:
                px_o = int(10 * (ox[i]+10))
                py_o = int(10 * (oy[i]+10))
                px_c = int(10 * (center_x+10))
                py_c = int(10 * (center_y+10))

                l = drawing.bresenham([px_c,py_c], [px_o,py_o])
                for j in range(len(l.path)):
                    lpx, lpy = l.path[j][0], l.path[j][1]
                    if (0 <= lpx < self.xw and 0 <= lpy < self.yw): # pixel in map
                        if j < (len(l.path)-1): # points on the way
                            self.datamap[lpx][lpy] += 0.01
                        else:   # end point
                            self.datamap[lpx][lpy] += 20

                        if self.datamap[lpx][lpy] > 10:
                            self.pmap[lpx][lpy] = 100   # black
                        else:
                            self.pmap[lpx][lpy] = 0
        return self.pmap

def main():

    """
    Example usage
    """
    print(__file__, "start")
    length = 20
    xyreso = 0.1  # x-y grid resolution
    xyw = int(round(length/xyreso))
    mapping = Mapping(xyw, xyw, xyreso)
    lidar_num = 200

    for i in range(10):
        center_x = 3 - i*0.3
        center_y = 3 - i*0.3
        ang = np.linspace(0,2*math.pi,lidar_num)
        dist = np.random.rand(lidar_num)*1+5
        ox = np.sin(ang) * dist # + center_x
        oy = np.cos(ang) * dist # + center_y
        pmap = mapping.update(ox, oy, center_x, center_y)

        rospy.init_node("map_pub")

        map_pub = rospy.Publisher('/slam_map',OccupancyGrid,queue_size=1)

        map_msg = OccupancyGrid()
        map_msg.header.seq = 1
        map_msg.header.stamp = rospy.Time().now()
        map_msg.header.frame_id = "map"

        map_msg.info.map_load_time = rospy.Time().now()
        map_msg.info.resolution = xyreso
        map_msg.info.width = xyw
        map_msg.info.height = xyw
        map_msg.info.origin.position.x = -xyw*xyreso/2.0
        map_msg.info.origin.position.y = -xyw*xyreso/2.0
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = list(pmap.reshape(-1)*100)
        
        map_pub.publish(map_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    main()     
