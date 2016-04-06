#!/usr/bin/env python

import rospy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from rbe3002.srv import *
# Add additional imports for each of the message types used

#Turtlebot Dimension Constants
wheel_rad = 3.5 / 100.0 #cm to m
wheel_base = 23.0 / 100.0 #cm to m


#Publishes Twist messages
def pubTwist(lin_Vel, ang_Vel):
    global pub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub.publish(msg)

#callback from rviz nav goal
#creates a path and uses that path to move to the location
def navToPose(goal):
    #get path from A*
    astar = getPath(pose, goal.pose)
    path = astar.path
    print "started driving"

    #drive to each waypoint in the path
    for p in path.poses:
        print "naving to pose"
        goToPose(p)


#drives to the pose of goal
def goToPose(goal):
    print "starting navigation!"

    #get current pose and orientation
    curT = pose.orientation.z
    curX = pose.position.x
    curY = pose.position.y

    assert isinstance(goal,PoseStamped)

    #get goal pose and orientation
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    goalT = yaw
    goalX = goal.pose.position.x
    goalY = goal.pose.position.y

    dX = goalX - curX
    dY = goalY - curY

    print "rotate!"
    rotateTo(.6, math.atan2(dY,dX))

    print "move!"
    driveStraight(.3, math.sqrt(dX**2 + dY**2))
    
    print "rotate!"
    rotateTo(.6, goalT)

    print "done"
    pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(.5,.60)
    rotateBy(-.5,math.radians(90))
    driveStraight(.5,.45)
    rotateBy(.5,math.radians(135))



#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    u = (u1 + u2) / 2
    w = (u1 - u2) / wheel_base

    start = rospy.Time().now().secs

    #publish a Twist message that moves until desired time, then publish a stop Twist message
    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()): # waits for said time and checks for ctrl C
        pubTwist(u,w) #publishes the move_msg
    pubTwist(0,0)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, dist):

    global pose

    #initialize position
    initX = pose.position.x
    initY = pose.position.y
    atGoal = False

    while (not atGoal and not rospy.is_shutdown()):
        curX = pose.position.x
        curY = pose.position.y
        curDis = math.sqrt((initX - curX)**2 + (initY - curY)**2)
        atGoal = (curDis >= dist)

        #keep going at given speed
        pubTwist(speed,0)
        rospy.sleep(0.1)

    #stop when goal is reached
    pubTwist(0,0)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraightAccel(speed, dist):

    numTimeSteps = 18

    #initialize position
    initX = pose.position.x
    initY = pose.position.y
    atGoal = False
    curSpeed = 0

    timeStep = (dist/speed)/numTimeSteps
    accel = speed/(numTimeSteps/3)

    while (not atGoal and not rospy.is_shutdown()):
        curX = pose.position.x
        curY = pose.position.y
        curDist = math.sqrt((initX - curX)**2 + (initY - curY)**2)
        atSpeed = (curDist >= speed)
        atGoal = (curDist >= dist)
        deccelDist = curSpeed*timeStep - .5*accel*timeStep**2
        if ((dist - curDist) <= deccelDist and (curSpeed - accel) >= 0):
            curSpeed -= accel
        elif (not atSpeed):
            curSpeed += accel
        pubTwist(curSpeed,0)
        rospy.sleep(timeStep)

        print curSpeed

    #stop when goal is reached
    pubTwist(0,0)


#Accepts an angle and makes the robot rotate around it. Assume there's no reason for
def rotateTo(angVel,angle):

    global pose

    timeRes = 0.1

    #initialize position
    atGoal = False

    while (not atGoal and not rospy.is_shutdown()):
        print "starting to rotate"
        curDelta = math.atan2(math.sin(pose.orientation.z - angle),math.cos(pose.orientation.z - angle))
        atGoal = (abs(curDelta) <= abs(angVel*timeRes))
        print curDelta
        print atGoal

        #keep going at given speed
        if (curDelta <= 0):
            pubTwist(0,angVel)
        else:
            pubTwist(0,-angVel)
        rospy.sleep(timeRes)

    #stop when goal is reached
    pubTwist(0,0)


#Accepts an angle and makes the robot rotate around it. Assume there's no reason for
def rotateBy(angVel,angle):

    global pose

    #initialize position
    initAng = pose.orientation.z
    atGoal = False

    while (not atGoal and not rospy.is_shutdown()):
        curAng = pose.orientation.z
        curDelta = abs(math.atan2(math.sin(curAng - initAng),math.cos(curAng - initAng)))
        atGoal = (curDelta >= angle)

        #keep going at given speed
        pubTwist(0,angVel)
        rospy.sleep(0.1)

    #stop when goal is reached
    pubTwist(0,0)


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global pose

    #initialize position
    initAng = pose.orientation.z
    atGoal = False

    while (not atGoal and not rospy.is_shutdown()):
        curAng = pose.orientation.z
        curDelta = curDelta = abs(math.atan2(math.sin(curAng - initAng),math.cos(curAng - initAng)))
        atGoal = (curDelta >= angle)

        print (str(curDelta) + "  " + str(angle))

        #keep going at given speed
        pubTwist(speed,speed/radius)
        rospy.sleep(0.2)

    #stop when goal is reached
    pubTwist(0,0)


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print "Bumper pressed!"
        executeTrajectory()


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    global start_pub

    assert isinstance(pose, Pose)
    assert isinstance(event, Odometry)
    pose.position.x = event.pose.pose.position.x
    pose.position.y = event.pose.pose.position.y

    rot_in_quat = event.pose.pose.orientation
    rot_array = [rot_in_quat.x,rot_in_quat.y,rot_in_quat.z,rot_in_quat.w]
    roll, pitch, yaw = euler_from_quaternion(rot_array)
    pose.orientation.z = yaw

    sendPose = PoseStamped()
    sendPose.header.frame_id = 'map'  
    sendPose.pose = event.pose.pose

    pose_pub.publish(sendPose)

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    global odom_tf
    global odom_list
    global start_pub
    global getPath

    pose = Pose()

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, None, queue_size=10) # Publisher for commanding robot motion
    #start_pub = rospy.Publisher('/rviz_start',PoseWithCovarianceStamped,queue_size = 1)
    #bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    odom_sub = rospy.Subscriber('/odom',Odometry,timerCallback,queue_size=1)
    goal_sub = rospy.Subscriber('/this_is_rviz', PoseStamped, navToPose, queue_size=1)
   
    getPath = rospy.ServiceProxy('astar', CalcPath)

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))



    print "Starting navigation node"


    rospy.spin()

    print "Ended node"
