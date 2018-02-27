#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from MovementManager import MovementManager
from rotate import Rotate
from std_msgs.msg import Float32
from waitable import Waitable
from cam import ObjectDetector

class MoveToRedObject(Waitable):

    MAX_DISTANCE_FROM_OBJECT = 0.25
    ARM_LENGTH = 0.25

    def __init__(self):
        rospy.loginfo("Distance to red")
        self.moving=False
        #self.distance_sub_=rospy.Subscriber("object_distance",Float32,self.move_to_red)
	super(MoveToRedObject,self).__init__()
	self.move_to_red()

    def move_to_red(self):
        """
        :param msg: msg of type Float32, distance from red object
        """
        object_detector = ObjectDetector()
        res = object_detector.get_distance_from_red_object()
        if res is None:
            rospy.loginfo("Red object was not detected")
            return

        distance, angle = res
        #if self.moving:
        #    self.force_close()
        #    return

        #distance = msg.data
        rospy.loginfo("distance from object : {}".format(distance/1000.0))
        distance = max(0, distance/1000.0-self.MAX_DISTANCE_FROM_OBJECT-self.ARM_LENGTH)
        rospy.loginfo("dis to red object : {}".format(distance))

        if angle > 180:
            angle = angle - 360

        Rotate(angle).wait_for_cb()
        self.moving=True
        MovementManager(distance, check_obstacles=True, threshold_direction=True).wait_for_cb()


if __name__ == '__main__':
    move = MoveToRedObject()
    rospy.spin()
