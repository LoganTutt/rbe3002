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
    global reachedGoal

    nav.navBot.rotateTo(-math.pi)
    nav.navBot.rotateTo(0)

    print "starting search"

    waypoint = getNextWaypoint()
    while waypoint and not rospy.is_shutdown():
        goal_pub.publish(waypoint)
        reachedGoal = False
        rospy.sleep(.01)
        while reachedGoal and not rospy.is_shutdown():
            pass
        waypoint = getNextWaypoint()

    print "finished exploring map"


def mapCallback(data_map)
    global global_map
    global_map = Grid(data_map.info.width, data_map.map_info.height, data_map.data, data_map.header.frame_id)


def statusCallback(status):
    global reachedGoal

    statusVal = status.something
    reachedGoal = statusVal == 3

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
    map_sub = rospy.Subscriber('/map',OccupancyGrid,mapCallback, queue_size=1)


    exploreMap()

if __name__ == '__main__':
    run()
