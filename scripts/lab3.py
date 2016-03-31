#!/usr/bin/env python

import rospy, tf, math
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

WHL_BASE = .23 #meter
WHL_RAD = .035 #meter


def aStar(start,goal):






#updates map 
def mapCallback(data_map):
    global robot_map
    pass

#updates odom data of robot
def odomCallback(odom_data):
    global pose
    assert isinstance(pose, Pose)

    #store pose data from odometry data
    pose.position.x = odom_data.pose.pose.position.x
    pose.position.y = odom_data.pose.pose.position.y
    
    quat = odom_data.pose.pose.orientation
    
    rotations = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(rotations)
    pose.orientation.z = yaw



if __name__ == '__main__':
    
    rospy.init_node('lab_3_node')

    global pose
    pose = Pose()

    grid_sub = rospy.Subscriber('/slam_gmapping/map',OccupancyGrid,mapCallback, queue_size = 1) 

    odom_sub = rospy.Subscriber('/odom',Odometry, odomCallback, queue_size = 1)
