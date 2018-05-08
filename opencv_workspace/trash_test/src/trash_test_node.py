#!/usr/bin/python

"""
Citation: http://blog.justsophie.com/smile-detection-using-opencv-designing-ros-node/

I referenced Sophie Li's ROS blog (link above) as a guide to write this node.
"""

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from copy import deepcopy

class trashDetector(object):

	def __init__(self):
		rospy.init_node("trash_detector_node", anonymous=True)
		self.trash_status_pub = rospy.Publisher('trash_detector/status', Bool, queue_size = 10)

		self.bridge_object = CvBridge()
		self.image_sub = None
		self.check_sub = rospy.Subscriber("search_with_cv", Bool, self.check_callback)
		self.frame = None

		# cascade
		self.cups_cascade = cv2.CascadeClassifier("~/catkin_ws/src/trashbot/opencv_workspace/data/cascade.xml")

	def image_callback(self, data):
		print("called img calback")
		try:
			#bgr8 is default opencv encoding
			self.frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

	def check_callback(self, data):
		print("called check callback")
		r = rospy.Rate(10)
		while (data == True):
			self.img_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
			r.sleep()
		self.img_sub.shutdown()
		self.img_sub = None


	def detect_trash(self):
		if self.frame == None:
			return

		gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
		cups = self.cups_cascade.detectMultiScale(gray, scaleFactor=1.2, minSize=(20, 20))

		# found trash
		if len(cups):
			self.trash_status_pub.publish(True)
		else:
			self.trash_status_pub.publish(False)
		#	print("found no trash")

	def run(self):
		# Detector will run at 10Hz just because
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.detect_trash() 			# check for trash
			r.sleep()						# sleeeeeeeeeeeep


def main():
	trash_detector = trashDetector()
	try:
		trash_detector.run()
	except KeyboardInterrupt:
		print("Closing...")

if __name__ == '__main__':
	main()
