#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from waitable import Waitable

class MovementManager(Waitable):

    STOP = 0
    STOP_MOVEMENT_THRESHOLD = 0.1
    SLOWDOWN_THRESHOLD = 0.2
    OBSTACLE_THRESHOLD = 0.75
    SPEED = 0.1

    def __init__(self, distance, check_obstacles=True, obstacle_threshold=None, threshold_direction=True, laser_width=0.4):
        #rospy.init_node('checkObstacle', anonymous=True)
        #rospy.loginfo("start movement manager")
        if obstacle_threshold is None:
            self.obstacle_threshold = self.OBSTACLE_THRESHOLD
        else:
            self.obstacle_threshold = obstacle_threshold
        self.laser_width = laser_width
        self.threshold_direction = threshold_direction
        self.laser_sub_ = rospy.Subscriber("/scan", LaserScan, self.look_for_obstacle)
        self.odom_sub_ = rospy.Subscriber("/odometry/filtered", Odometry, self.calc_covered_distance)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.range_ahead = distance
        self.distance = distance
        self.fast_pace_speed = self.SPEED if distance > 0.45 else self.SPEED * (2.0/3)
        self.move_function = self.move_forward if distance > 0.45 else self.simple_move_forward
        self.speed = self.fast_pace_speed
        self.starting_point = None
        self.initial_cords = None
        self.check_obstacles=check_obstacles
        super(MovementManager, self).__init__()

    def simple_move_forward(self):
        print 'in simple_move_forward: speed:{}'.format(self.speed)
        msg = Twist()
        msg.linear.x = self.speed

    def stop(self):
        self.speed = self.STOP
        self.odom_sub_.unregister()
        self.laser_sub_.unregister()

    def simple_move_forward(self):
        print 'in simple_move_forward: speed:{}'.format(self.speed)
        msg = Twist()
        if self.range_ahead < distance * 0.95:
            msg.linear.x = self.speed
        else:
            self.stop()
        self.pub.publish(msg)

    def move_forward(self):
        """
        Publish data to /cmd_vel in order to manage robot movement. Since the robot accelerates, the speed is changed with
        consideation of covered distance, as the robot slows down relatively to the left distance.
        """
        print 'in move_forward: speed:{}'.format(self.speed)
        msg = Twist()

        if self.range_ahead < self.STOP_MOVEMENT_THRESHOLD:
            self.speed = self.STOP
            self.odom_sub_.unregister()
            self.laser_sub_.unregister()

        elif self.range_ahead < self.SLOWDOWN_THRESHOLD:
            # As the range_ahead becomes smaller within every iteration, the robot will slow down after it passed a predefined part of the distance
            self.speed = self.fast_pace_speed * (1 - ( self.distance - self.range_ahead)/ self.distance)
        msg.linear.x = self.speed
	rospy.loginfo("current speed : {}".format(self.speed))
	self.pub.publish(msg)

    def calc_covered_distance(self, odom):
        """
        Update left range ahead given odom data
        :param odom: odometry data
        """
        odom_position = odom.pose.pose.position
        if self.initial_cords is None :
            self.initial_cords = (odom_position.x, odom_position.y)

        self.covered_distance = math.hypot(self.initial_cords[0] - odom_position.x, self.initial_cords[1] - odom_position.y)
        self.range_ahead = min(self.range_ahead, (self.distance - self.covered_distance))
        rospy.loginfo("covered distance : {}, range ahead : {}, {}".format(self.covered_distance, self.range_ahead, self.initial_cords))
        if self.speed != self.STOP:
            self.move_function()

    def look_for_obstacle(self, data):
        """
        Update range from obstacle given data from laser
        :param data: laser data
        """
        if not self.check_obstacles:
            self.laser_sub_.unregister()
            return

        ranges_threshold = int(self.laser_width * len(data.ranges))
        laser_ranges= data.ranges[ranges_threshold:-ranges_threshold]
        range_from_obstacle = min(laser_ranges)
        rospy.loginfo("range_from_obstacle: {}".format(range_from_obstacle))
        if (self.threshold_direction and range_from_obstacle < self.obstacle_threshold) or (not self.threshold_direction and range_from_obstacle > self.obstacle_threshold):
	    self.stop()
            rospy.loginfo("resetting distance, range from obstacle : {}, threshold: {}".format(range_from_obstacle, self.obstacle_threshold))
            #range_from_obstacle = 0
            self.range_ahead = min(self.range_ahead, range_from_obstacle)
            rospy.loginfo("range ahead: {}".format(self.range_ahead))


