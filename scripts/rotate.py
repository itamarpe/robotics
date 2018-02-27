#!/usr/bin/env python
import math
import tf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from waitable import Waitable

TEST_ANG = 400
DEG_PER_SEC = 10

def degrees_to_radians(deg):
    return (deg * math.pi) / 180

class Rotate(Waitable):

    def __init__(self, angle, laser_threshold=0, threshold_direction=True, laser_width=0.4, range_center=True):
	rospy.loginfo("init called")
        self.odom_sub_ = rospy.Subscriber("/odometry/filtered", Odometry, self.calc_angular_movement)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	if laser_threshold != 0 :
		rospy.loginfo("laser")
		self.laser_scan = rospy.Subscriber("/scan", LaserScan, self.look_for_obstacle)
		self.obstacle_threshold = laser_threshold
		self.threshold_direction = threshold_direction
		self.laser_width = laser_width
        self.initial_euler_z= None
        self.move = True
        fixed_angle = math.fabs(angle)
        self.direction = -1 * (angle / fixed_angle)
        self.angle = degrees_to_radians(fixed_angle)
        self.initial_speed = degrees_to_radians(min(DEG_PER_SEC, fixed_angle * 0.4))
        self.min_speed = degrees_to_radians(5)
        self.speed = self.initial_speed
        self.last_euler_z = None
        self.distance_traveled = 0
	self.range_center = range_center
        super(Rotate, self).__init__()

    def perform_angular_movement(self):
        msg = Twist()
        msg.angular.z = self.direction * self.speed
        rospy.loginfo("current angular speed: {}".format(self.speed))
        self.pub.publish(msg)

    def get_range_from_obstacle(self, data):
	if self.range_center:
	        range_from_obstacle = min(data.ranges[int(self.laser_width * len(data.ranges)): -int(self.laser_width * len(data.ranges))])
	else:
	        #range_from_obstacle = min(data.ranges[:int(self.laser_width*len(data.ranges))]+data.ranges[-int(self.laser_width*len(data.ranges)):])
		range_from_obstacle = min(data.ranges[350:400])
	return range_from_obstacle

    def look_for_obstacle(self, data):
	rospy.loginfo('len: {}'.format(len(data.ranges)))
        range_from_obstacle = self.get_range_from_obstacle(data) 
        rospy.loginfo("range_from_obstacle: {}".format(range_from_obstacle))
        if (self.threshold_direction and range_from_obstacle < self.obstacle_threshold) or (not self.threshold_direction and range_from_obstacle > self.obstacle_threshold):
            self.speed = 0
            self.odom_sub_.unregister()
            self.laser_scan.unregister()
	    self.perform_angular_movement()
            rospy.loginfo("resetting distance, range from obstacle : {}".format(range_from_obstacle))
            #range_from_obstacle = 0

    def canonicalize_euler_cords(self, z_cord):
        return z_cord + 2*math.pi if z_cord < 0 else z_cord


    def calc_angular_movement(self, odom):
        """
        Calculate current angular movement data, will be used when we should decide whether the robot should perform another movement
        :param odom:
        """
        odom_orientation = odom.pose.pose.orientation
        cords = (odom_orientation.x, odom_orientation.y, odom_orientation.z, odom_orientation.w)
        euler_cords = tf.transformations.euler_from_quaternion(cords)
        euler_z = self.canonicalize_euler_cords(euler_cords[2])

        if self.initial_euler_z is None:
            self.initial_euler_z = euler_z
            self.last_euler_z = euler_z
            rospy.loginfo("intial euler z: {}".format(euler_z))
            return

        delta = math.fabs(self.last_euler_z - euler_z)
        self.last_euler_z = euler_z

        if delta > self.speed:
            delta = 2 * math.pi - delta

        self.distance_traveled += delta

        if self.distance_traveled < self.angle:
            if (self.distance_traveled / self.angle) < 0.9:
                self.speed = max(self.initial_speed * (1 - (self.distance_traveled / self.angle) ** 2), self.min_speed)
            else:
                self.speed = 0
            self.perform_angular_movement()
            rospy.loginfo("last: {}, traveled: {}, current euler z : {}".format(self.last_euler_z, self.distance_traveled, euler_z))

        else:
            self.odom_sub_.unregister()
            rospy.loginfo('done rotating')


if __name__ == '__main__':
    rospy.init_node('Rotata', anonymous=True)
    move = Rotate(TEST_ANG)
    rospy.spin()
