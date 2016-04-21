#!/usr/bin/env python

import rospy, planner, nav, tf, math
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point as ROSPoint
from actionlib_msgs.msg import GoalStatusArray, GoalStatus


def getNextFrontier():
    startPoint = planner.pose2point(nav.navBot.cur,gobal_map)
    curNode = Node(startPoint,initOri,startPoint,None)
    nodes = {curNode.key: curNode}
    frontier = [frontier]
    nextFrontier = []
    
    while frontier:
        for node in frontier:
            if(global_map.getVal(curNode.point.x,curNode.point.y) != -1):
                front = expandFrontier(node)
                if front:
                    return front
            nodes[node.key()] = node
            nextFrontier.append(curNode.createNewNodes(nodes,global_map,75))
        frontier = nextFrontier

    print "No Frontiers found"
    return None

        
        
def expandFrontier(Node start):
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
        if curNode:
            curNode = curFrontier[0]

    if len(frontier) > .4/global_map.map_info.resolution:
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
        goal_pub.publish(waypoint)
        reachedGoal = False
        rospy.sleep(.01)
        while reachedGoal and not rospy.is_shutdown():
            pass
        waypoint = getNextFrontier()

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


    status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback, queue_size=1)
    map_sub = rospy.Subscriber('/map',OccupancyGrid,mapCallback, queue_size=1)


    exploreMap()

if __name__ == '__main__':
    run()
