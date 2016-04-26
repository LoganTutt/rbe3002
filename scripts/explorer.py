#!/usr/bin/env python

import rospy, planner, nav, tf, math
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point as ROSPoint
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from nodes import Grid, Node, Point

def getNextFrontier():
    startPoint = planner.pose2point(nav.navBot.cur.pose,global_map)
    curNode = Node(startPoint,1,startPoint,None)
    nodes = {curNode.key(): curNode}
    frontier = [curNode]
    nextFrontier = []
    
    while frontier:
        for node in frontier:
            if(global_map.getVal(node.point.x,node.point.y) == -1):
                front = expandFrontier(node)
                if front:
                    print "Found frontier"
                    return front
            nodes[node.key()] = node
            nextFrontier.append(node.createNewNodes(nodes,global_map,75))
        frontier = nextFrontier
        nextFrontier = []

    print "No Frontiers found"
    return None


def getNextWaypoint():
    node = getNextFrontier()
    if node:
        return planner.node2pose(node,global_map)
    return None
        
        
def expandFrontier(start):
    nodes = {start.key(): start}
    fullFrontier = [start]
    curFrontier = [start]
    while curFrontier:
        curNode = curFrontier[0]
        for node in curNode.createNewNodes(nodes,global_map,75):
          if global_map.getValFromPoint(node.point) == -1:
              for tempNode in node.createNewNodes(nodes,global_map,75):
                  if tempNode != -1:
                      nodes[node.key()] = node
                      curFrontier.append(node)
                      break
        fullFrontier.append(curNode)
        curFrontier.remove(curNode)
        if curFrontier:
            curNode = curFrontier[0]

    if len(fullFrontier) > .4/global_map.map_info.resolution:
        return start
    else:
        return None


def exploreMap():
    global reachedGoal

    nav.navBot.rotateTo(-math.pi/2)
    nav.navBot.rotateTo(-math.pi)
    nav.navBot.rotateTo(math.pi/2)
    nav.navBot.rotateTo(0)

    print "starting search"

    waypoint = getNextWaypoint()
    while waypoint and not rospy.is_shutdown():
        #goal_pub.publish(waypoint)
        print "Navigating to: " + str(waypoint.pose.position.x) +","+str(waypoint.pose.position.y)
        nav.navToPose(waypoint)
        waypoint = getNextWaypoint()

    print "finished exploring map"


def mapCallback(data_map):
    global global_map
    global_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info, data_map.header.frame_id)


def run():
    global goal_pub
    global frontier_pub


    rospy.init_node('exploration_node')
    navType = rospy.get_param('~nav',default = 'rbe')
    if navType != 'rbe': rospy.delete_param('~nav')
    if navType == 'rbe':
        print "Using rbe nav"
        goal_pub = rospy.Publisher('/rbe_goal',PoseStamped,queue_size=1)
    else:
        print "using gMapping nav"
        goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)

    planner.init()
    nav.init()

    frontier_pub =rospy.Publisher('/frontier/frontier',GridCells,queue_size=1) 


    #status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback, queue_size=1)
    map_sub = rospy.Subscriber('/map',OccupancyGrid,mapCallback, queue_size=1)


    exploreMap()

if __name__ == '__main__':
    run()
