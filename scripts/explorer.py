#!/usr/bin/env python

import rospy, planner, nav, tf, math
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point as ROSPoint
from actionlib_msgs.msg import GoalStatusArray, GoalStatus


def getNextFrontier():
    pass


def getNextWaypoint():
    pass


def exploreMap():
    nav.navBot.rotateTo(-math.pi)
    nav.navBot.rotateTo(0)

    print "starting search"

    waypoint = getNextWaypoint()
    while waypoint and not rospy.is_shutdown():
        goal_pub.publish(waypoint)
        rospy.sleep(.01)
        while reachedGoal and not rospy.is_shutdown():
            pass
        waypoint = getNextWaypoint()

    print "finished exploring map"

def statusCallback(status):
    pass

def run():
    global goal_pub
    global frontier_pub


    rospy.init_node('exploration_node')
    navType = rospy.get_param('~nav',default = 'gMap')
    if navType != 'gMap': rospy.delete_param('~nav')
    if navType == 'rbe':
        print "Using rbe nav"
        goal_pub = rospy.Publisher('/rbe_goal',PoseStamped,queue_size=1)
    else:
        print "using gMapping nav"
        goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
        planner.init()
        nav.init()

    frontier_pub =rospy.Publisher('/frontier/frontier',GridCells,queue_size=1) 


    status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback, queue_size=1)


    exploreMap()

if __name__ == '__main__':
    run()
