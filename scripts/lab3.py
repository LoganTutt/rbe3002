#!/usr/bin/env python

import rospy, tf, math
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

WHL_BASE = .23 #meter
WHL_RAD = .035 #meter


#calculates and returns path to the goal point
#start and goal should be pose objects
def aStar(start,goal):
    pass






#updates map 
def mapCallback(data_map):
    global robot_map
    pass

def run():

    rospy.init_node('lab_3_node')

    global pose
    pose = Pose()

    grid_sub = rospy.Subscriber('/map',OccupancyGrid,mapCallback, queue_size = 1) 


if __name__ == '__main__':
    

