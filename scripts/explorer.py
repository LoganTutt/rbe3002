#!/usr/bin/env python

import rospy, planner, nav, tf, math, copy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point as ROSPoint
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from nodes import Grid, Node, Point
from Queue import Queue

def getNextFrontier():
    startPoint = planner.pose2point(nav.navBot.cur.pose,global_map)
    curNode = Node(startPoint,1,startPoint,None)
    nodes = {curNode.key(): curNode}
    frontier = Queue()
    frontier.put(curNode)
    nextFrontier = []
    tempMap = copy.deepcopy(global_map)
    cells = GridCells()
    cells.cell_height = global_map.map_info.resolution
    cells.cell_width = global_map.map_info.resolution
    cells.header.frame_id = global_map.frame_id
    cells.header.stamp = rospy.Time(0)
    
    while frontier:
        node = frontier.get()
        if tempMap.getValFromPoint(node.point) == -1:
            front = expandFrontier(node)
            if front:
                print "found Frontier"
                return front
            continue
        tempNodes = node.createNewNodes(nodes,tempMap,75)
        for n in tempNodes:
            nodes[n.key()] = n
            p = ROSPoint()
            tPose = planner.node2pose(n,global_map)
            p.x=tPose.pose.position.x
            p.y = tPose.pose.position.y
            cells.cells.append(p)
            frontier_pub.publish(cells)

            frontier.put(n)

#    while frontier:
#        for node in frontier:
#            #if node is unexplored
#            if(tempMap.getVal(node.point.x,node.point.y) == -1):
#                front = expandFrontier(node)
#                #if the frontier is big enough
#                if front:
#                    print "Found frontier"
#                    return front
#                break
#            tempNodes = node.createNewNodes(nodes,tempMap,75)
#            #add new nodes
#            for n in tempNodes:
#                nodes[n.key()] = n
#                p = ROSPoint()
#                tPose = planner.node2pose(n,global_map)
#                p.x=tPose.pose.position.x
#                p.y = tPose.pose.position.y
#                cells.cells.append(p)
#                frontier_pub.publish(cells)
#
#            nextFrontier.extend(tempNodes)
#        frontier = nextFrontier
#        nextFrontier = []

    print "No Frontiers found"
    return None


def getNextWaypoint():
    node = getNextFrontier()
    if node:
        return planner.node2pose(node,global_map)
    return None
        
def expandFrontier(start):
    print "expanding frontier"
    nodes = {start.key(): start}
    fullFrontier = []
    curFrontier = [start]
    endPoints = []
    cells = GridCells()
    cells.cell_height = global_map.map_info.resolution
    cells.cell_width = global_map.map_info.resolution
    cells.header.frame_id = global_map.frame_id
    cells.header.stamp = rospy.Time(0)
    while curFrontier:
        curNode = curFrontier[0]
        for node in curNode.createNewNodes(nodes,global_map,75):
            if global_map.getValFromPoint(node.point) == -1 and node.orientation%1==0:
                for tempNode in node.createNewNodes(nodes,global_map,101):
                    if global_map.getValFromPoint(tempNode.point) > 75:
                        endPoints.append(tempNode)
                        break
                    if global_map.getValFromPoint(tempNode.point) != -1:
                        curFrontier.append(node)
                        nodes[node.key()] = node
                        break
        p = ROSPoint()
        tPose = planner.node2pose(curNode,global_map)
        p.x=tPose.pose.position.x
        p.y = tPose.pose.position.y
        cells.cells.append(p)
        frontier_pub.publish(cells)
        fullFrontier.append(curNode)
        curFrontier.remove(curNode)


    print "len = " + str(len(fullFrontier))
    if len(fullFrontier) > .4/global_map.map_info.resolution:
        if len(endPoints) >= 2:
            print "found bounded edge"
            tempNode =  Node(Point((endPoints[1].point.x+endPoints[0].point.x)/2,(endPoints[1].point.y+endPoints[0].point.y)/2),1,start.point,None)
            if planner.global_map.getValFromPoint(tempNode.point) > 60:
                print "blocked node"
                return None
        else:
            print "found unbounded edge"
            return start
    else:
        print "no Frontier"
        return None


def exploreMap():
    global reachedGoal

    nav.navBot.rotateTo(-math.pi/2)
    rospy.sleep(1)
    nav.navBot.rotateTo(-math.pi)
    rospy.sleep(1)
    nav.navBot.rotateTo(math.pi/2)
    rospy.sleep(1)
    nav.navBot.rotateTo(0)
    rospy.sleep(5)

    print "starting search"

    waypoint = getNextWaypoint()
    while waypoint and not rospy.is_shutdown():
        goal_pub.publish(waypoint)
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
