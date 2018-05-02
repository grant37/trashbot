import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class trashDetector(object):

	def __init__(self):

		self.bridge_object = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)

	def camera_callback(self, data):

		try:
			#bgr8 is default opencv encoding
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)


# cropping image
height, width, channels = cv_image.shape
decenter = 160
rows_to_watch = 20
crop_img = cv_image[(height)/2 + decenter:(height)/2 + (decenter + rows_to_watch)][1:width]

hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

# tracking color, color picker web tool
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([50, 255, 255])

#idk
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

def main():
	trash_detector = trashDetector()
	rospy.init_node('trash_detecting_node', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Closing...")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
