#!/usr/bin/env python

import rospy, planner,tf
import from geometry_msgs import Pose
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point as ROSPoint


def run():
    global goal_pub
    global frontier_pub


    rospy.init_node('exploration_node')
    navType = rospy.get_param('/nav',default='gMap')
    if navType == 'rbe':
        goal_pub = rospy.Publisher('/rbe_goal',PoseStamped,queue_size=1)
    else:
        goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped.queue_size=1)
        planner.run()
        pass

if __name__ == '__main__':
    run()
