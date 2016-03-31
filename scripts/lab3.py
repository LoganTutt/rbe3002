#!/usr/bin/env python

import rospy, tf, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

WHL_BASE = .23 #meter
WHL_RAD = .035 #meter

#POOP


if __name__ == '__main__':
    
    rospy.init_node('lab_3_node')

