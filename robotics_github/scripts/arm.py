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
	self.computed_path = False
	rospy.loginfo("Start arm controller")	

    def receive_cords(self, cartesian_point):
	self.cartesian_point = cartesian_point
	
    def move_arm_forward(self):
	rospy.loginfo("Moving arm forward")
	if self.computed_path == True:
		rospy.loginfo("Skip, already computed path")
		return
	while self.cartesian_point is None:
		rospy.loginfo("Waiting for cartesian point")
		time.sleep(0.5)

	points = []
        points.append(self.arm.get_current_pose().pose)
        dest_pose = Pose()
        dest_pose.orientation.w = 1
        #dest_pose.position.x = points[0].position.x + 0.1
        #dest_pose.position.y = points[0].position.y 
        #dest_pose.position.z = points[0].position.z 
        dest_pose.position.x = self.cartesian_point.x
        dest_pose.position.y = points[0].position.y
        dest_pose.position.z = self.cartesian_point.z
        points.append(copy.deepcopy(dest_pose))
	rospy.loginfo("Current position : {}, {}, {}".format(points[0].position.x, points[0].position.y, points[0].position.z))
	rospy.loginfo("Dest position : {}, {}, {}".format(points[1].position.x, points[1].position.y, points[1].position.z))
       	rospy.loginfo("Computing path")
	(plan, fraction) = self.arm.compute_cartesian_path(points, 0.01, 0.0)
	self.computed_path = True
	#rospy.loginfo(plan)
	rospy.loginfo("Finished computing path")
        move_res = self.arm.execute(plan)
	if move_res :
		rospy.loginfo("Finished moving arm")
	else:
		rospy.loginfo("Failed moving arm")
	moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.init_node('moveArm', anonymous=True)
    controller = ArmController()
    robot = moveit_commander.RobotCommander()


