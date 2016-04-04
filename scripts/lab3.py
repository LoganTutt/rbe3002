#!/usr/bin/env python

import rospy, tf, math, nodes
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

WHL_BASE = .23 #meter
WHL_RAD = .035 #meter



#converts a pose (Pose) to a point (Point) in the world grid
def pose2point(pose):

    assert isinstance(pose, Pose)

    dX = pose.position.x - map_conversion[0]
    dY = pose.position.y - map_conversion[1]

    return Point(int(dX/map_conversion[2]), int(dY/map_conversion[2]))




#calculates and returns path to the goal point
#start and end are poses
#passes back a list of grid wayPoints to get from star to end
def aStar(start, goal):
    global cost_map

    # convert from poses to points + init orientation (1,2,3, or 4)
    startPoint = pose2point(start)
    goalPoint = pose2point(goal)
    initEuler = euler_from_quaternion(
        [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w])
    initYaw = initEuler[2]  # returns the yaw

    # convert from euler yaw to 1,2,3,4, as used by node
    initOri = round(initYaw / (math.pi / 2)) + 1
    if initOri <= 0: initOri += 4


    #A* here
    curNode = Node(startPoint, initOri, goalPoint, None)
    nodes = {curNode.key: curNode}
    frontier = [curNode]

    #keep searching the frontiers based on the lowest cost
    while (not curNode.point.equals(goalPoint)):
        nodeKids = curNode.createNewNodes(nodes, robot_map, 50)

        for kid in nodeKids:
            # add the nodes to frontier based on cost
            for ind in range(0,len(frontier)):
                if (kid.cost < node.cost):
                    frontier.insert(ind,kid)
                    break
                elif (ind == len(frontier) - 1): frontier.append(kid)

            # add the new node to the dictionary of nodes
            nodes[kid.key()] = kid

            #add kids' costs to the cost_map
            cost_map.setVal(kid.point.x, kid.point.y, kid.cost)

        publishCostMap()

        frontier.remove(curNode)
        curNode = frontier[0]

    #order and optimize the waypoints
    wayPoints = []

    while (not curNode.prevNode == None):
        if (not curNode.orientation == curNode.prevNode.orientation): wayPoints.insert(0,node2pose(curNode))
        curNode = curNode.prevNode

    return wayPoints



def node2pose(node):

    pose = Pose()

    pose.position.x = node.point.x * map_conversion[0]
    pose.position.y = node.point.y * map_conversion[1]

    # convert to quaternian
    tempOri = curNode.orientation
    if (tempOri >= 3): tempOri -= 4
    tempOri -= 1
    nodeYaw = tempOri * (math.pi / 2)

    pose.orientation.z = quaternion_from_euler(0, 0, nodeYaw)

    return pose



#updates map 
def mapCallback(data_map):
    global robot_map
    global map_info
    global map_conversion
    global cost_map

    assert isinstance(data_map,OccupancyGrid)
    assert isinstance(robot_map, Grid)
    assert isinstance(cost_map, Grid)

    robot_map = Grid(data_map.info.width, data_map.info.height, data_map.data)
    cost_map = Grid(data_map.info.width, data_map.info.height, [0]*len(data_map.data))

    map_info = data_map

    map_conversion = [data_map.info.origin.position.x, data_map.info.origin.position.y, data_map.info.resolution]



#testing for now
def pathCallback(goalStamped):

    goal = poseSt.pose

    origin = Pose()
    origin.position.x = 0
    origin.position.y = 0
    origin.orientation.z = quaternion_from_euler(0,0,0)

    aStar(origin, goal)



def publishCostMap():

    costGrid = OccupancyGrid()

    costGrid.header.frame_id = 'map'
    costGrid.info = map_info
    costGrid.data = cost_map.data

    costMap_pub.publish(costGrid)



def run():

    rospy.init_node('lab_3_node')

    global map_conversion
    global map_info
    global robot_map
    global cost_map
    global costMap_pub
    global pose

    pose = Pose()

    grid_sub = rospy.Subscriber('/map',OccupancyGrid, mapCallback, queue_size = 1)
    costMap_pub = rospy.Publisher('/robot_cost_map',OccupancyGrid, queue_size = 1)
    path_sub = rospy.Subscriber('/rviz_goal', PoseStamped, pathCallback, queue_size=1)


if __name__ == '__main__':
    
    run()
