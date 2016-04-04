#!/usr/bin/env python

import rospy, tf, math
from node import *
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

WHL_BASE = .23 #meter
WHL_RAD = .035 #meter



#converts a pose (Pose) to a point (Point) in the world grid
def poseToPoint(pose):

    global robot_map
    global map_info

    assert isinstance(pose, Pose)

    dX = pose.position.x - map_info[0]
    dY = pose.position.y - map_info[1]

    return Point(int(dX/map_info[2]), int(dY/map_info[2]))




#calculates and returns path to the goal point
#start and end should be an point representing the grid square
def aStar(start,goal):








#updates map 
def mapCallback(data_map):
    global robot_map
    global map_info
    assert isinstance(data_map,OccupancyGrid)
    robot_map = data_map.data
    map_info = [data_map.info.origin.position.x, data_map.info.origin.position.y, data_map.info.resolution]



def run():

    rospy.init_node('lab_3_node')

    global map_info
    global robot_map
    global pose
    pose = Pose()

    grid_sub = rospy.Subscriber('/map',OccupancyGrid,mapCallback, queue_size = 1) 


if __name__ == '__main__':
    

