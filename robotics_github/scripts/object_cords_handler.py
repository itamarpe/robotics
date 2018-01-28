#!/usr/bin/env python
from math import pi
import rospy
import tf
import copy
import moveit_msgs.msg
import moveit_commander
import time
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, PoseStamped, Vector3
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs import point_cloud2 as pc2
from ex1.msg import Circle


class ObjectCordsHandler(object):

    def __init__(self):
	self.red_object_cords = None
	self.red_obj_subcriber = rospy.Subscriber("red_object_coordinates", Circle, self.receive_red_object_cords)
	self.pcl_subscriber = rospy.Subscriber("/softkinetic_camera/depth/points", PointCloud2, self.handle_pointcloud_image)	
	self.cords_publisher = rospy.Publisher("cartesian_points_publisher", Vector3, queue_size=10)
	self.cartesian_point = None
	rospy.loginfo("Starting object cords handler")

    def receive_red_object_cords(self,data):
	self.red_object_cords = (data.center_x, data.center_y)

    def handle_pointcloud_image(self,data):
	if self.cartesian_point != None:
	    return
	while self.red_object_cords is None:
 	    rospy.loginfo("Waiting for red object cords")
	    time.sleep(0.1)
	image = list(pc2.read_points(data,skip_nans = False,field_names=("x","y","z","rgb")))
	red_object_x, red_object_y = self.red_object_cords[0], self.red_object_cords[1]
	rospy.loginfo("object cords {},{}".format(red_object_x, red_object_y))
	rospy.loginfo("image dim {},{}".format(data.width, data.height))
	pcl_point = image[data.width * red_object_y + red_object_x]
	self.cartesian_point = Vector3(pcl_point[0],pcl_point[1],pcl_point[2])
	rospy.loginfo("Cartesian point : {}".format(self.cartesian_point))
	self.cords_publisher.publish(self.cartesian_point)
	time.sleep(10)

if __name__ == '__main__':
    rospy.init_node('objectCords', anonymous=True)
    objectCordsHandler = ObjectCordsHandler()
    rospy.spin()
