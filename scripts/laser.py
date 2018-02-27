#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from waitable import Waitable

THRESHOLD = 0.25

def look_for_obstacle(data):
    ranges_threshold = int(0.4*len(data.ranges))
    laser_ranges= data.ranges[ranges_threshold:-ranges_threshold] 
    #range_from_obstacle = min(laser_ranges)
    range_from_obstacle = min(data.ranges)
    """
    range_from_obstacle = min(data.ranges)
    range_from_obstacle = min(data.ranges[:int(0.2*len(data.ranges))]+data.ranges[-int(0.2*len(data.ranges)):])
    range_from_obstacle = min(data.ranges[450:500])
    print "{}, {}".format(range_from_obstacle, data.ranges.index(range_from_obstacle))
    left_range = data.ranges[:int(0.2*len(data.ranges))]
    print left_range
    """
    rospy.loginfo("range_from_obstacle: {}".format(range_from_obstacle))


def main():
    rospy.init_node('laser_test')
    laser_sub = rospy.Subscriber("/scan", LaserScan, look_for_obstacle)
    rospy.spin()


if __name__ == "__main__":
    main()
