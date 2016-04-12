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

class Navigate:
    start = Pose()
    goal = Pose()
    cur = Pose()
    goalAngle = 0.0
    topAngVel = 0.0
    topLinVel = 0.0
    angVel = 0.0
    linVel = 0.0

    # PID Constants
    turnKp = 0.75
    turnKi = 0.000175
    turnKd = 0.0
    driveKp = 0.25
    driveKi = 0.0
    driveKd = 0.0

    curTurnDelta = 0.0
    prevTurnDelta = 0
    totalTurnDelta = 0

    def __init__(self, topAngVel, topLinVel):
        self.topAngVel = topAngVel
        self.topLinVel = topLinVel

    #Publishes Twist messages
    def pubTwist(self):
        global pub
        msg = Twist()
        msg.linear.x = self.linVel
        msg.angular.z = self.angVel
        pub.publish(msg)


    #drives to the pose of goal
    def goToPose(self, goal):

        self.start = self.cur
        self.goal = goal.pose

        #get current pose and orientation
        curX = self.cur.position.x
        curY = self.cur.position.y

        assert isinstance(self.goal,Pose)

        #get goal pose and orientation
        quat = self.goal.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        goalT = yaw
        goalX = self.goal.position.x
        goalY = self.goal.position.y

        dX = goalX - curX
        dY = goalY - curY

        self.rotateTo(math.atan2(dY,dX))

        self.driveStraight(math.sqrt(dX**2 + dY**2))


    def getCurrentAngle(self):

        quat = self.cur.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        return yaw


    #This function accepts a speed and a distance for the robot to move in a straight line
    def driveStraight(self, dist):

        #initialize position
        initX = self.cur.position.x
        initY = self.cur.position.y
        atGoal = False

        while (not atGoal and not rospy.is_shutdown()):
            curX = self.cur.position.x
            curY = self.cur.position.y
            curDis = math.sqrt((initX - curX)**2 + (initY - curY)**2)
            atGoal = (curDis >= dist)

            #keep going at given speed
            self.linVel = self.topLinVel #TODO PID
            rospy.sleep(0.1)

        #stop when goal is reached
        self.linVel = 0.0
        self.resetPID()


    #Accepts an angle and makes the robot rotate around it. Assume there's no reason for
    def rotateTo(self, angle):
        self.goalAngle = angle
        timeRes = 0.1

        #initialize position
        atGoal = False

        while (not atGoal and not rospy.is_shutdown()):
            self.prevTurnDelta = self.curTurnDelta
            self.curTurnDelta = abs(math.atan2(math.sin(self.getCurrentAngle() - self.goalAngle),math.cos(self.getCurrentAngle() - self.goalAngle)))
            self.totalTurnDelta += self.curTurnDelta
            atGoal = (abs(self.curTurnDelta) <= abs(self.topAngVel*timeRes)/2)

            self.linVel = 0.0
            rospy.sleep(timeRes)

        #stop when goal is reached
        print "got to the angle"
        self.resetPID()


    #Accepts an angle and makes the robot rotate around it. Assume there's no reason for
    def rotateBy(self, angle):

        #initialize position
        initAng = self.cur.orientation.z
        atGoal = False

        while (not atGoal and not rospy.is_shutdown()):
            curAng = self.cur.orientation.z
            self.curTurnDelta = abs(math.atan2(math.sin(curAng - initAng),math.cos(curAng - initAng)))
            atGoal = (self.curTurnDelta >= angle)

            #keep going at given speed
            self.linVel = 0.0
            rospy.sleep(0.1)

        #stop when goal is reached
        self.resetPID()


    def updatePID(self, event):

        print str(self.getCurrentAngle()) + "  " + str(self.goalAngle)

        timeDelta = .01 #abs(event.current_real - event.last_real)
        curDelta = math.atan2(math.sin(self.goalAngle - self.getCurrentAngle()), math.cos(self.goalAngle - self.getCurrentAngle()))
        self.totalTurnDelta += curDelta * timeDelta
        self.angVel = (self.turnKp * curDelta + self.turnKi * self.totalTurnDelta - self.turnKd * abs(self.prevTurnDelta - curDelta) / timeDelta)
        self.prevTurnDelta = curDelta

        self.pubTwist()


    def resetPID(self):
        self.totalTurnDelta = 0
        self.prevTurnDelta = 0


# Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print "Bumper pressed!"
        # do bumper stuff here


# handles the current position of the robot
def odomCallback(event):
    global pose_pub
    global navBot

    assert isinstance(navBot, Navigate)
    assert isinstance(event, Odometry)

    navBot.cur.position.x = event.pose.pose.position.x
    navBot.cur.position.y = event.pose.pose.position.y
    navBot.cur.orientation = event.pose.pose.orientation

    sendPose = PoseStamped()
    sendPose.header.frame_id = 'map'
    sendPose.pose = event.pose.pose

    pose_pub.publish(sendPose)


# creates a path and uses that path to move to the location
def navToPose(goal):
    # get path from A*
    astar = getPath(navBot.cur, goal.pose)
    path = astar.path
    print "started driving"

    # drive to each waypoint in the path
    for p in path.poses:
        navBot.goToPose(p)
        print "naving to pose"


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global odom_tf
    global odom_list
    global start_pub
    global getPath

    global navBot

    navBot = Navigate(.6, .3)

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, None, queue_size=10)
    #start_pub = rospy.Publisher('/rviz_start',PoseWithCovarianceStamped,queue_size = 1)
    #bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    odom_sub = rospy.Subscriber('/odom',Odometry,odomCallback,queue_size=1)
    goal_sub = rospy.Subscriber('/this_is_rviz', PoseStamped, navToPose, queue_size=1)
   
    getPath = rospy.ServiceProxy('global_path', CalcPath)

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    rospy.Timer(rospy.Duration(.01), navBot.updatePID)


    print "Starting navigation node"
    rospy.spin()

    print "Ended node"
