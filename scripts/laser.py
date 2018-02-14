#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from waitable import Waitable


def look_for_obstacle(data):
    ranges_threshold = int(0.4*len(data.ranges))
    laser_ranges= data.ranges[ranges_threshold:-ranges_threshold]
    range_from_obstacle = min(laser_ranges)
    rospy.loginfo("range_from_obstacle: {}".format(range_from_obstacle))


def main():
    rospy.init_node('laser_test')
    laser_sub = rospy.Subscriber("/scan", LaserScan, look_for_obstacle)
    rospy.spin()


if __name__ == "__main__":
    main()
