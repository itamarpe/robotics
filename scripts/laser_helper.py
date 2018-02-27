#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan

RANGE_THRESHOLD = 0.25

class LaserHelper(object):

    def __init__(self):
        rospy.init_node('laserHelper', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.check_laser_margins)

    def get_margin_min(self, data, margin_threshold):
        margin_min = float("inf")
 	for i in range(2):
		range = data.range[int(len(data.range)*margin_threshold):]
		margin_min = min(range)
		sleep(0.5)
	return margin_min

    def get_min_right(self, data):
	return get_margin_min(data, RANGE_THRESHOLD)

    def get_min_left(self, data):
	return get_margin_min(data, -RANGE_THRESHOLD)

    def check_laser_margins(self, data):
        min_left = self.get_min_left(data)
	min_right = self.get_min_right(data)
        self.range_ahead = min(self.range_ahead, range_from_obstacle)

if __name__ == '__main__':
    laser_helper = LaserHelper()
    rospy.spin()
