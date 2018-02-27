#!/usr/bin/env python
import rospy
import tf
import cv2
import time
import math
from ex1.msg import Circle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from waitable import Waitable

class ObjectDetector(Waitable):


    def __init__(self):
        self.image_clr_sub_ = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.handle_image_color)
        self.image_depth_sub_= rospy.Subscriber("/kinect2/qhd/image_depth_rect",Image,self.handle_image_depth)
        self.red_sub_ = rospy.Subscriber("red_object_coordinates",Circle,self.get_red_image_cords)
	self.kinect_points_sub_ = rospy.Subscriber("/kinect2/qhd/points", PointCloud2, self.get_angle)
        self.publisher = rospy.Publisher("image_data",Image,queue_size=10)
        self.distance_publisher = rospy.Publisher("object_distance",Float32,queue_size=10)
        self.image_depth = None
        self.distance = None
        self.callback_called = False
	self.angle = None
        rospy.loginfo("init")
        super(ObjectDetector, self).__init__()

    def get_red_image_cords(self, cords):
        """
        Get the coordinates of the red image, then publish them to required topic
        :param cords: cords of the red image
        """
        self.cords = cords
	if cords.radius == 0:
            rospy.loginfo("No red object was found")
            self.distance_publisher.publish(None)
            self.distance = False
            self.callback_called = True
            return
    	if self.image_depth is not None:
            distance_from_red = self.image_depth[cords.center_y, cords.center_x]
       	    self.distance_publisher.publish(distance_from_red)
            self.distance = distance_from_red
        self.callback_called = True
        rospy.loginfo("Distance from red object : {}".format(distance_from_red))

    def get_angle(self, image_data):
	while not self.callback_called:
	    time.sleep(0.1)
	gen = pc2.read_points(image_data, skip_nans=False, field_names=("x","y","z"))
	points_list = list(gen)
	if self.cords.radius == 0:
	    self.angle=False
	    return
	rospy.loginfo("{},{}".format(image_data.width, image_data.height))
	red_object_center = points_list[self.cords.center_y * image_data.width + self.cords.center_x]
	x,y,z = red_object_center[0], red_object_center[1], red_object_center[2]
	angle = math.atan(x)
	angle = angle*180/math.pi
	if angle < 0:
	    angle = 360+angle
	rospy.loginfo("{},{},{}".format(x,y,z))
	self.angle = angle
	rospy.loginfo("Angle : {}".format(angle))
	

    def get_distance_from_red_object(self):
        while not self.callback_called or not self.angle:
            time.sleep(0.1)
	self.force_close()
        if not self.distance:
            return None
        return (self.distance, self.angle)


    def handle_image_color(self, image_data):
        """
        Receive colored image, publish it s.t red image finder can find the coordinates of the red object
        :param image_data: colored image data
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publisher.publish(image_data)
            rate.sleep()

    def handle_image_depth(self, image_data):
        """
        Receive data which contains a 540x960 matrix that stores depth data per each matrix cell, i.e, image coordinate
        :param image_data: depth image data
        """
        cv_bridge = CvBridge()
        self.image_depth = cv_bridge.imgmsg_to_cv2(image_data)

if __name__ == '__main__':
    rospy.init_node("object_detector")
    object_detector = ObjectDetector()
    rospy.spin()

