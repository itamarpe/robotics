#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


MAX_LASER = 3
MAX_DISTANCE = 0.5
DISTANCE_TO_OBJECT = 0.5

class MoveToObstacle(object):

    SLOW_PACE = 0.1
    FAST_PACE = SLOW_PACE * 4
    STOP = 0
    SLOWDOWN_THRESHOLD = 0.01

    def __init__(self):
        rospy.init_node('checkObstacle', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.look_for_obstacle)
        rospy.Subscriber("/odometry/filtered", Odometry, self.calc_covered_distance)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.range_ahead = MAX_DISTANCE
        self.driving = False
        self.speed = self.FAST_PACE
        self.starting_point = None
        self.initial_cords = None

    def laser_callback(self, data):
        self.speed = self.FAST_PACE
        rospy.loginfo("finished initialization")

    def move_forward(self):
        msg = Twist()
        self.driving = True

        # if self.range_ahead < (MAX_DISTANCE * self.SLOWDOWN_THRESHOLD) or self.speed < self.SLOWDOWN_THRESHOLD:
        if self.range_ahead < self.SLOWDOWN_THRESHOLD * DISTANCE_TO_OBJECT:
            self.speed = self.STOP
        else:
            # As the range_ahead becomes smaller within every iteration, the robot will slow down
            self.speed = self.FAST_PACE * (1 - ( DISTANCE_TO_OBJECT - self.range_ahead)/ DISTANCE_TO_OBJECT) ** 1.1
        rospy.loginfo("current speed : {}".format(self.speed))
        msg.linear.x = self.speed
        self.pub.publish(msg)

    def calc_covered_distance(self, odom):
        """
        Update left range ahead given odom data
        :param odom:
        """
        odom_position = odom.pose.pose.position
        if self.initial_cords is None :
            self.initial_cords = (odom_position.x, odom_position.y)

        self.covered_distance = math.hypot(self.initial_cords[0] - odom_position.x, self.initial_cords[1] - odom_position.y)
        self.range_ahead = min(self.range_ahead, (MAX_DISTANCE - self.covered_distance))
        rospy.loginfo("covered distance : {}, range ahead : {}, {}".format(self.covered_distance, self.range_ahead, self.initial_cords))
        self.move_forward()

    def look_for_obstacle(self, data):
        """
        Update range from obstacle given data from laser
        :param data: laser input
        """
        range_from_obstacle = min(data.ranges)
        if range_from_obstacle < MAX_DISTANCE :
            range_from_obstacle = 0
        self.range_ahead = min(self.range_ahead, range_from_obstacle)
        #self.move_forward()

        rospy.loginfo("range ahead, driving: {}, {}".format(self.range_ahead, self.driving))

    def odom_callback(self, msg):
      if self.starting_point is None:
        self.starting_point = msg.twist.twist.linear.x
        rospy.loginfo("odometry starting point: {}".format(self.starting_point))

if __name__ == '__main__':
    move = MoveToObstacle()
    rospy.spin()
