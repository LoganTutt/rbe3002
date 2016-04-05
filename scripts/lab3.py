#!/usr/bin/env python

import rospy, tf, math, nodes
from nodes import *
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point as ROSPoint
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
    quat = [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    initYaw = yaw  # returns the yaw

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
                if (kid.cost < frontier[ind].cost):
                    frontier.insert(ind,kid)
                    break
                elif (ind == len(frontier) - 1): frontier.append(kid)

            # add the new node to the dictionary of nodes
            nodes[kid.key()] = kid

            #add kids' costs to the cost_map
            cost_map.setVal(kid.point.x, kid.point.y, int(kid.cost))

        frontier.remove(curNode)
        curNode = frontier[0]

    #order and optimize the waypoints
    wayPoints = []
    path = GridCells()

    path.cell_height = map_info.resolution
    path.cell_width = map_info.resolution
    path.header.frame_id = 'map'
    path.header.stamp = rospy.get_rostime()

    while (not curNode.prevNode == None):
        if (not curNode.orientation == curNode.prevNode.orientation):
            wayPoints.insert(0,node2pose(curNode))
        rosPoint = ROSPoint()
        rosPoint.x = curNode.point.x * path.cell_width + map_info.origin.position.x
        rosPoint.y = curNode.point.y * path.cell_height + map_info.origin.position.y
        rosPoint.z = 0.0
        path.cells.append(rosPoint)
        curNode = curNode.prevNode

    path_pub.publish(path)
    return wayPoints



def node2pose(node):

    pose = Pose()

    pose.position.x = node.point.x * map_conversion[0]
    pose.position.y = node.point.y * map_conversion[1]

    # convert to quaternian
    tempOri = node.orientation
    if (tempOri >= 3): tempOri -= 4
    tempOri -= 1
    nodeYaw = tempOri * (math.pi / 2)

    quat = quaternion_from_euler(0, 0, nodeYaw)

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose



#subscriber callbacks
def mapCallback(data_map):
    global robot_map
    global map_info
    global map_conversion
    global cost_map

    assert isinstance(data_map,OccupancyGrid)

    robot_map = Grid(data_map.info.width, data_map.info.height, data_map.data)
    cost_map = Grid(data_map.info.width, data_map.info.height, [0]*len(data_map.data))

    map_info = data_map.info

    map_conversion = [data_map.info.origin.position.x, data_map.info.origin.position.y, data_map.info.resolution]

def pathCallback(goalStamped):
    global cost_map

    print "Got Heem"

    cost_map = Grid(cost_map.width, cost_map.height, [0]*len(cost_map.data))

    goal = goalStamped.pose

    dao = aStar(start_pose, goal)       #dao = way in Chinese

    way = Path()
    way.header = goalStamped.header
    for waypoint in dao:
        tempPoseSt = PoseStamped()
        tempPoseSt.pose = waypoint
        way.poses.append(tempPoseSt)

    way_pub.publish(way)

    print "findeh de path"

def startCallback(startPose):
    global start_pose
    print "set start"

    start_pose = startPose.pose.pose

    start_stamped_pose = PoseStamped()
    start_stamped_pose.pose = start_pose
    start_stamped_pose.header = startPose.header
    startPose_pub.publish(start_stamped_pose)


def publishCostMap():

    costGrid = OccupancyGrid()

    costGrid.header.frame_id = 'map'
    costGrid.info = map_info
    costGrid.data = cost_map.data

    #map cost_map to between 0 and 127 for fancy colors in rviz
    maxVal = max(cost_map.data)

    minVal = int('inf')
    for cost in cost_map.data:
        if (not (cost == 0) and (cost < minVal)): minVal = cost

    factor = 127/(maxVal - minVal)

    cost_map.data = [((i - minVal) * factor if (i != 0) else 0) for i in cost_map.data]

    costMap_pub.publish(costGrid)



def run():

    rospy.init_node('lab_3_node')

    global map_conversion
    global map_info
    global robot_map
    global cost_map
    global pose
    global start_pose

    global startPose_pub
    global costMap_pub
    global path_pub
    global way_pub

    pose = Pose()

    grid_sub = rospy.Subscriber('/map',OccupancyGrid, mapCallback, queue_size = 1)
    goal_sub = rospy.Subscriber('/rviz_goal', PoseStamped, pathCallback, queue_size=1)
    start_sub = rospy.Subscriber('/rviz_start', PoseWithCovarianceStamped, startCallback, queue_size=1)
    startPose_pub = rospy.Publisher('/robot_start', PoseStamped, queue_size=1)
    costMap_pub = rospy.Publisher('/robot_cost_map', OccupancyGrid, queue_size=1)
    path_pub = rospy.Publisher('/robot_path', GridCells, queue_size=1)
    way_pub = rospy.Publisher('/robot_waypoints', Path, queue_size=1)

    while not rospy.is_shutdown():
        if (cost_map != None):
            publishCostMap()
        rospy.sleep(.25)


if __name__ == '__main__':
    
    run()
