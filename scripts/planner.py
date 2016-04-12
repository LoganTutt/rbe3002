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
def pose2point(pose, grid):

    assert isinstance(pose, Pose)


    poseStamped = PoseStamped()
    poseStamped.pose = pose
    poseStamped.header.frame_id = 'odom'
    poseStamped.header.stamp = rospy.Time(0)

    pose = transformer.transformPose(grid.frame_id,poseStamped).pose
    

    dX = pose.position.x - grid.map_info.origin.position.x
    dY = pose.position.y - grid.map_info.origin.position.y

    return Point(int(dX/grid.map_info.resolution), int(dY/grid.map_info.resolution))



#converts a node object to a Pose object for use in a path
def node2pose(node,grid):

    pose = Pose()

    pose.position.x = ((node.point.x+.5) * grid.map_info.resolution)+grid.map_info.origin.position.x
    pose.position.y = ((node.point.y+.5) * grid.map_info.resolution)+grid.map_info.origin.position.y

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

    poseStamped = PoseStamped()
    poseStamped.pose = pose
    poseStamped.header.frame_id = grid.frame_id
    poseStamped.header.stamp = rospy.Time(0)

    pose = transformer.transformPose('odom',poseStamped).pose
    return pose



#calculates and returns path to the goal point
#start and end are poses
#passes back a list of Pose wayPoints to get from star to end
def aStar(start, goal, grid,wayPub):
    global cost_map
    
    # convert from poses to points + init orientation (1,2,3, or 4)
    startPoint = pose2point(start,grid)
    goalPoint = pose2point(goal,grid)
    quat = [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    initYaw = yaw  # returns the yaw
    
    cutoffVal = 60
    if grid.getVal(startPoint.x,startPoint.y) > cutoffVal:
        cutoffVal = grid.getVal(startPoint.x,startPoint.y)

    if grid.getVal(goalPoint.x,goalPoint.y) > cutoffVal:
        return None
   
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
        nodeKids = curNode.createNewNodes(nodes, grid, cutoffVal) #create new nodes that are neighbors to current node
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
        if not frontier:
            return None
        
        curNode = frontier[0] #curNode becomes the frontier node with the lowest cost

    #order and optimize the waypoints
    wayPoints = []
    path = GridCells()
    ways = GridCells()

    #path is a GridCells, and is used to display the path in rviz
    path.cell_height = grid.map_info.resolution
    path.cell_width = grid.map_info.resolution
    path.header.frame_id = grid.frame_id
    path.header.stamp = rospy.get_rostime()

    # ways is a GridCells, and is used to display the waypoints in rviz
    ways.cell_height = grid.map_info.resolution
    ways.cell_width = grid.map_info.resolution
    ways.header.frame_id = grid.frame_id
    ways.header.stamp = rospy.get_rostime()
    wayPoints.append(node2pose(curNode,grid))
    
    temp = ROSPoint()
    temp.x = (curNode.point.x+.5) * path.cell_width + grid.map_info.origin.position.x
    temp.y = (curNode.point.y+.5) * path.cell_height + grid.map_info.origin.position.y
    temp.z = path.cell_height * .25 #offset above costmap
    ways.cells.append(temp)

    curNode = curNode.prevNode

    distCount = 1.5/grid.map_info.resolution
    count = 0
    #generates waypoints at each rotation location
    while (curNode != None and not curNode.prevNode == None):
        if (not curNode.orientation == curNode.prevNode.orientation or count >= distCount) and curNode.prevNode.prevNode != None:
            wayPoints.insert(0,node2pose(curNode.prevNode,grid))
            temp = ROSPoint()
            temp.x = (curNode.prevNode.point.x+.5) * path.cell_width + grid.map_info.origin.position.x
            temp.y = (curNode.prevNode.point.y+.5) * path.cell_height + grid.map_info.origin.position.y
            temp.z = path.cell_height * .25 #offset above costmap
            ways.cells.append(temp)
            count = 0
        rosPoint = ROSPoint()
        rosPoint.x = (curNode.point.x+.5) * path.cell_width + grid.map_info.origin.position.x
        rosPoint.y = (curNode.point.y+.5) * path.cell_height + grid.map_info.origin.position.y
        rosPoint.z = path.cell_height * .125 #offset above path
        path.cells.append(rosPoint)
        curNode = curNode.prevNode
        count += 1

    path_pub.publish(path)
    wayPub.publish(ways)


    return wayPoints



#finds the path to goal from start and returns the waypoints to reach there
def calcWaypoints(start,goal, grid, wayPub):
    global cost_map

    print "Got start and goal poses"

    cost_map = Grid(grid.width, grid.height, [0]*len(grid.data), grid.map_info, grid.frame_id)

    dao = aStar(start, goal, grid, wayPub)       #dao = way in Chinese

    if dao == None: return

    way = Path()
    for waypoint in dao:
        tempPoseSt = PoseStamped()
        tempPoseSt.pose = waypoint
        way.poses.append(tempPoseSt)

    waypoints_pub.publish(way)
    return way

    print "findeh de path"



#subscriber callbacks

#data_map is an OccupancyGrid
#stores the incoming global map
def mapCallback(data_map):
    global robot_map
    global cost_map

    assert isinstance(data_map,OccupancyGrid)

    print "recieved map"
    robot_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info,data_map.header.frame_id)
    #cost_map = Grid(data_map.info.width, data_map.info.height, [0]*len(data_map.data),data_map.info)



#data_map is an OccupancyGrid
#stores the incoming local map
def localMapCallback(data_map):
    global local_map
    global cost_map

    assert isinstance(data_map,OccupancyGrid)

    print "recieved local map"
    local_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info,data_map.header.frame_id)



#service handler. Takes in a start and end pose then returns a path
def calcPath(req):
    global cost_map

    start = req.start
    goal = req.end
    path = calcWaypoints(start, goal, robot_map, global_way_pub)

    cost_map.publish(global_costmap_pub)
    cost_map=None

    
    return CalcPathResponse(path)


#service handler. Takes in a start and end pose then returns a path
def localCalcPath(req):
    start = req.start
    goal = req.end
    path = calcWaypoints(start,goal, local_map, way_pub)

    
    return CalcPathResponse(path)
    


def run():

    rospy.init_node('planning_node')

    global robot_map
    global cost_map

    #make publishers global so that they can be used anywhere
    global costMap_pub
    global path_pub
    global waypoints_pub
    global way_pub
    global global_costmap_pub
    global global_way_pub
    global transformer
    
    cost_map = None

    
    transformer = tf.TransformListener()

    #subscribers
    grid_sub = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid, mapCallback, queue_size = 1)
    local_grid_sub = rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid, localMapCallback, queue_size = 1)

    #publishers
    costMap_pub = rospy.Publisher('/robot_cost_map', OccupancyGrid, queue_size=1)
    global_costmap_pub = rospy.Publisher('/global_cost_map', OccupancyGrid, queue_size=1)
    path_pub = rospy.Publisher('/robot_path', GridCells, queue_size=1)
    way_pub = rospy.Publisher('/robot_waypoints', GridCells, queue_size=1)
    global_way_pub = rospy.Publisher('/global_waypoints', GridCells, queue_size=1)
    waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=1)

    global_serv = rospy.Service('global_path',CalcPath, calcPath)
    local_serv = rospy.Service('local_path',CalcPath, localCalcPath)

    rospy.sleep(1)
    print "Ready"

    #this handles updating the cost_map
    while not rospy.is_shutdown():
        if (cost_map != None):
            cost_map.publish(costMap_pub)
        rospy.sleep(.125)


if __name__ == '__main__':
    
    run()
