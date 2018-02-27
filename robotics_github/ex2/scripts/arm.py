#!/usr/bin/env python
from math import pi
import rospy
import tf
import copy
import moveit_msgs.msg
import moveit_commander
import time
import sys
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs import point_cloud2 as pc2
from ex1.msg import Circle


class ArmController(object):

    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm")
	self.object_cartesian_subscriber = rospy.Subscriber("cartesian_points_publisher", Vector3, self.receive_cords)
	self.cartesian_point = None
	rospy.loginfo("Start arm controller")	

    def receive_cords(self, cartesian_point):
	self.cartesian_point = cartesian_point
	
    def move_arm_forward(self):
	points = []
        points.append(self.arm.get_current_pose().pose)
        dest_pose = Pose()
        dest_pose.orientation.w = 1
        dest_pose.position.x = points[0].position.x + 0.2
        dest_pose.position.y = points[0].position.y + 0.2
        dest_pose.position.z = points[0].position.z + 0.2
        points.append(copy.deepcopy(dest_pose))
        (plan, fraction) = self.arm.compute_cartesian_path(points, 0.01, 0)
        self.arm.execute(plan)

	rospy.loginfo("Moving arm forward")
	while self.cartesian_point is None:
		rospy.loginfo("waiting for cartesian point")
		time.sleep(0.5)
	points = []
	points.append(self.arm.get_current_pose().pose)
	dest_pose = Pose()
	dest_pose.orientation.w = 1
	dest_pose.position.x = -0.00989#self.cartesian_point.x
	dest_pose.position.y = 0.2385#self.cartesian_point.y
	dest_pose.position.z = 0.46680#self.cartesian_point.z
	points.append(copy.deepcopy(dest_pose))
	rospy.loginfo("Computing path")
	(plan, fraction) = self.arm.compute_cartesian_path(points, 0.01, 0)
	rospy.loginfo("Finished computing path")
	rospy.loginfo(plan)
	self.arm.execute(plan)
	

if __name__ == '__main__':
    rospy.init_node('moveArm', anonymous=True)
    controller = ArmController()
    robot = moveit_commander.RobotCommander()
    #controller.move_arm()
    #rospy.spin()


