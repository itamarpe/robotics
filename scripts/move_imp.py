#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from MovementManager import MovementManager

class MoveToObstacle(object):

    DISTANCE_TO_OBJECT = 0.5

    def __init__(self):
	"""
	Move forward using MovementManager, using a predefined distance of 0.5m
	"""
	rospy.init_node("checkObstacle", anonymous=True)
	movement_manager = MovementManager(self.DISTANCE_TO_OBJECT)

if __name__ == '__main__':
    move = MoveToObstacle()
    rospy.spin()
