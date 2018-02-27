#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from waitable import Waitable
from ex1.msg import Circle

class MoveArm(Waitable):

    def __init__(self):
	#self.image_clr_sub_ = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.handle_image_color)
	#self.image_depth_sub_= rospy.Subscriber("/kinect2/qhd/image_depth_rect",Image,self.handle_image_depth)
	#self.image_publisher = rospy.Publisher("image_data", Image, queue_size=10)
	self.scan_sub = rospy.Subscriber("/softkinetic_camera/depth/points", PointCloud2, self.move)
	self.red_obj_sub = rospy.Subscriber("red_object_coordinates",Circle, self.manage_cords)
	self.cords = False 	
	self.image_subscriber = rospy.Subscriber("/softkinetic_camera/depth/image_raw", Image, self.handle_image_color)
	self.f = False
		
    def handle_image_color(self, image_data):
	"""
	rospy.loginfo("test")
	rospy.loginfo(image_data.shape)
	"""
	rate = rospy.Rate(1)
	
	if not self.f:
		rospy.loginfo(image_data)
		self.f = True

	while not rospy.is_shutdown():
		self.image_publisher.publish(image_data)
		rospy.loginfo(image_data)
		rate.sleep()
	

    def move(self, scan_data):
	while self.cords is None:
		time.sleep(0.1)
	data = pc2.read_points(scan_data, field_names=("x","y","z"), skip_nans=False)
	data = list(data)
	rospy.loginfo("{} {} {}".format(len(data), scan_data.width, scan_data.height))
	
    def manage_cords(self, cords):
	if cords.radius == 0 :
		self.cords = False
	else:
		self.cords = (cords.center_x, cords.center_y)



if __name__ == '__main__':
    rospy.init_node("move_arm")    
    arm = MoveArm()
    rospy.spin()

