#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


MAX_LASER = 3
DISTANCE_TO_OBJECT = 0.5

class MoveToObstacle(object):

    SLOW_PACE = 0.4
    FAST_PACE = 1
    STOP = 0
    SLOWDOWN_THRESHOLD = 0.1

    def __init__(self):
        rospy.init_node('checkObstacle', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.range_ahead = None
        self.driving = False
        self.speed = self.FAST_PACE

    def callback(self, data):
	print data.pose.pose
	return
        msg = Twist()
#        if not self.range_ahead:
        self.range_ahead = min(data.ranges)
        self.driving = True

        if self.range_ahead <= DISTANCE_TO_OBJECT * (1 + self.SLOWDOWN_THRESHOLD):
            self.speed = self.STOP

        elif DISTANCE_TO_OBJECT * (1 + self.SLOWDOWN_THRESHOLD) < self.range_ahead < MAX_LASER:
            self.speed = self.SLOW_PACE * (self.range_ahead - (DISTANCE_TO_OBJECT * (1 + self.SLOWDOWN_THRESHOLD)))

        if self.driving:
            if self.speed <= self.STOP + self.SLOWDOWN_THRESHOLD :
                self.driving = False

            msg.linear.x = self.speed
            self.pub.publish(msg)

        rospy.loginfo("range ahead, driving:{}, {}, {}".format(self.range_ahead, self.speed, self.driving))

if __name__ == '__main__':
  move = MoveToObstacle()
  rospy.spin()
