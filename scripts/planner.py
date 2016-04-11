#!/usr/bin/env python

import rospy, tf, math, nodes, copy
from nodes import *
from nav import navToPose
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point as ROSPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rbe3002.srv import *




#converts a pose (Pose) to a point (Point) in the world grid
def pose2point(pose):

    assert isinstance(pose, Pose)

    dX = pose.position.x - map_conversion[0]
    dY = pose.position.y - map_conversion[1]

    return Point(int(dX/map_conversion[2]), int(dY/map_conversion[2]))




#calculates and returns path to the goal point
#start and end are poses
#passes back a list of Pose wayPoints to get from star to end
def aStar(start, goal, grid):
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

    print str(curNode.point.x) + "," + str(curNode.point.y)

    #keep searching the frontiers based on the lowest cost until goal is reached
    while (not curNode.point.equals(goalPoint)):
        nodeKids = curNode.createNewNodes(nodes, grid, 25) #create new nodes that are neighbors to current node
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
        print len(frontier)
        curNode = frontier[0] #curNode becomes the frontier node with the lowest cost

    #order and optimize the waypoints
    wayPoints = []
    path = GridCells()
    ways = GridCells()

    #path is a GridCells, and is used to display the path in rviz
    path.cell_height = map_info.resolution
    path.cell_width = map_info.resolution
    path.header.frame_id = 'map'
    path.header.stamp = rospy.get_rostime()

    # ways is a GridCells, and is used to display the waypoints in rviz
    ways.cell_height = map_info.resolution
    ways.cell_width = map_info.resolution
    ways.header.frame_id = 'map'
    ways.header.stamp = rospy.get_rostime()
    wayPoints.append(node2pose(curNode))

    distCount = 2.0/map_conversion[2]
    count = 0
    #generates waypoints at each rotation location
    while (not curNode.prevNode == None):
        if (not curNode.orientation == curNode.prevNode.orientation or count >= distCount):
            wayPoints.insert(0,node2pose(curNode.prevNode))
            temp = ROSPoint()
            temp.x = curNode.prevNode.point.x * path.cell_width + map_info.origin.position.x
            temp.y = curNode.prevNode.point.y * path.cell_height + map_info.origin.position.y
            temp.z = path.cell_height * .25 #offset above costmap
            ways.cells.append(temp)
            count = 0
        rosPoint = ROSPoint()
        rosPoint.x = curNode.point.x * path.cell_width + map_info.origin.position.x
        rosPoint.y = curNode.point.y * path.cell_height + map_info.origin.position.y
        rosPoint.z = path.cell_height * .125 #offset above path
        path.cells.append(rosPoint)
        curNode = curNode.prevNode
        count += 1

    path_pub.publish(path)
    way_pub.publish(ways)

    return wayPoints


#converts a node object to a Pose object for use in a path
def node2pose(node):

    pose = Pose()

    pose.position.x = (node.point.x * map_conversion[2])+map_conversion[0]
    pose.position.y = (node.point.y * map_conversion[2])+map_conversion[1]

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

#data_map is an OccupancyGrid
#stores the incoming map and other useful information
def mapCallback(data_map):
    global robot_map
    global map_info
    global map_conversion
    global cost_map

    assert isinstance(data_map,OccupancyGrid)

    print "recieved map"
    robot_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info)
    cost_map = Grid(data_map.info.width, data_map.info.height, [0]*len(data_map.data),data_map.info)

    map_info = data_map.info

    map_conversion = [data_map.info.origin.position.x, data_map.info.origin.position.y, data_map.info.resolution]

#goalStamped is a PoseStamped
#finds the path to goalStamped and returns the waypoints to reach there
def pathCallback(goalStamped, grid):
    global cost_map

    print "Got start and goal poses"


    cost_map = Grid(cost_map.width, cost_map.height, [0]*len(cost_map.data), cost_map.map_info)

    goal = goalStamped.pose #get the pose from the stamped pose

    dao = aStar(start_pose, goal, grid)       #dao = way in Chinese

    way = Path()
    way.header = goalStamped.header
    for waypoint in dao:
        tempPoseSt = PoseStamped()
        tempPoseSt.header = goalStamped.header
        tempPoseSt.pose = waypoint
        way.poses.append(tempPoseSt)

    waypoints_pub.publish(way)
    return way

    print "findeh de path"

#startPose is a PoseWithCovarianceStamped
#stores the start pose of for path planning and prints the location for rviz
def startCallback(startPose):
    global start_pose

    start_pose = startPose.pose.pose #get the pose from the stamped pose

    start_stamped_pose = PoseStamped()
    start_stamped_pose.pose = start_pose
    start_stamped_pose.header = startPose.header
    startPose_pub.publish(start_stamped_pose)


##publishes the cost_map
#def publishCostMap():
#    global cost_map
#    costGrid = OccupancyGrid()
#
#
#    costGrid.header.frame_id = 'map'
#    costGrid.info = map_info
#    temparr = copy.deepcopy(cost_map.data) #protect ourselves from volitility
#
#    #map cost_map to between 0 and 127 for fancy colors in rviz
#    maxVal = max(temparr)
#
#    minVal = float('inf')
#    for cost in temparr:
#        if (not (cost == 0) and (cost < minVal)): minVal = cost
#
#    factor = 100.0/(maxVal - minVal)
#
#    if(maxVal == minVal): return
#
#    # costGrid.data = [(int((i - minVal) * factor) if (i != 0) else 0) for i in cost_map.data]
#    costGrid.data = []
#    for i in temparr:
#        if(i!=0):
#            costGrid.data.append(int((i - minVal) * factor))
#        else:
#            costGrid.data.append(0)
#
#    costMap_pub.publish(costGrid)

#service handler. Takes in a start and end pose then returns a path
def calcPath(req):
    start = PoseWithCovarianceStamped()
    start.pose.pose = req.start
    startCallback(start)

    goal = PoseStamped()
    goal.pose = req.end
    path = pathCallback(goal, robot_map)

    return CalcPathResponse(path)


    


def run():

    rospy.init_node('planning_node')

    global map_conversion
    global map_info
    global robot_map
    global cost_map
    global pose
    global start_pose

    #make publishers global so that they can be used anywhere
    global startPose_pub
    global costMap_pub
    global path_pub
    global waypoints_pub
    global way_pub

    pose = Pose()
    
    cost_map = None

    #subscribers
    grid_sub = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid, mapCallback, queue_size = 1)
    #local_grid_sub = rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid, localMapCallback, queue_size = 1)
    #goal_sub = rospy.Subscriber('/rviz_goal', PoseStamped, pathCallback, queue_size=1)
    #start_sub = rospy.Subscriber('/rviz_start', PoseWithCovarianceStamped, startCallback, queue_size=1)

    #publishers
    startPose_pub = rospy.Publisher('/robot_start', PoseStamped, queue_size=1)
    costMap_pub = rospy.Publisher('/robot_cost_map', OccupancyGrid, queue_size=1)
    path_pub = rospy.Publisher('/robot_path', GridCells, queue_size=1)
    way_pub = rospy.Publisher('/robot_waypoints', GridCells, queue_size=1)
    waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=1)

    global_serv = rospy.Service('global_path',CalcPath, calcPath)
    local_serv = rospy.Service('local_path',CalcPath, calcPath)

    rospy.sleep(1)
    print "Ready"

    #this handles updating the cost_map
    while not rospy.is_shutdown():
        if (cost_map != None):
            cost_map.publish(costMap_pub)
        rospy.sleep(.125)


if __name__ == '__main__':
    
    run()
