#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import math
import numpy as np
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"

AUTONOMOUS_MODE = True

TURN_ANGLE = 15
KP = 0.0008

COLORS = ["r", "y", "b"]


class driveStop(object):
	"""class that will help the robot drive and stop at certain conditions
	"""
	def __init__(self):
		"""initalize the node"""
		rospy.init_node("driveStop")
		self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
                self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 2)
		rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

		""" initialize the box dimensions"""
                self.flag_colors_box = [((0,0),(0,0)),((0,0),(0,0))]
		self.flag_center = (0,0)
		self.flag_size = 0
                self.areas = []
                self.centers = []
                self.state = 0

                """driving stuff"""
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 3
		self.cmd.drive.steering_angle = 0

		"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)
                self.imagePush = None

                self.bridge = CvBridge()
		self.min_value=0
                self.switchTime = -1

	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
		rPix_Width = self.flag_colors_box[0][1][0] - self.flag_colors_box[0][0][0]
		rPix_Height = self.flag_colors_box[0][1][1] - self.flag_colors_box[0][0][1]

		yPix_Width = self.flag_colors_box[1][1][0] - self.flag_colors_box[1][0][0]
		yPix_Height = self.flag_colors_box[1][1][1] - self.flag_colors_box[1][0][1]

		bPix_Width = self.flag_colors_box[2][1][0] - self.flag_colors_box[2][0][0]
		bPix_Height = self.flag_colors_box[2][1][1] - self.flag_colors_box[2][0][1]


		self.rBoxCenter = (((self.flag_colors_box[0][0][0] + self.flag_colors_box[0][1][0]) / 2), ((self.flag_colors_box[0][0][1] + self.flag_colors_box[0][1][1]) / 2))
                self.yBoxCenter = (((self.flag_colors_box[1][0][0] + self.flag_colors_box[1][1][0]) / 2), ((self.flag_colors_box[1][0][1] + self.flag_colors_box[1][1][1]) / 2))
		self.bBoxCenter = (((self.flag_colors_box[2][0][0] + self.flag_colors_box[2][1][0]) / 2), ((self.flag_colors_box[2][0][1] + self.flag_colors_box[2][1][1]) / 2))

		self.rArea = rPix_Width * rPix_Height
		self.yArea = yPix_Width * yPix_Height
		self.bArea = bPix_Width * bPix_Height

		self.areas = [self.rArea, self.yArea, self.bArea]
		self.centers = [self.rBoxCenter, self.yBoxCenter, self.bBoxCenter]

	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")

                self.data = data

		#applies the current filter to the image and stores the image in imagePush
                self.flag_colors_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image)

		#finds the size of the box
                self.size_calc()

		#outputs the image to the IMAGE_TOPIC
		try:
		    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
		except CvBridgeError as e:
                    print("Error bridging Zed image", e)

		if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass

	def drive(self):
                error = 0
		# print(self.state)
		# if self.state + 1 >= len(path):
		# 	pass
		# elif self.areas[COLORS.index(path[self.state + 1])] >= self.areas[COLORS.index(path[self.state])]:
		# 	if self.switchTime == -1:
		# 		self.state += 1
		# 		self.switchTime = time.time()
		# 	elif time.time() - self.switchTime > 2:
		# 		self.state += 1
		# 		self.switchTime = time.time()
                print("RED: " + str(self.areas[0]))
                print("YELLOW: " + str(self.areas[1]))
                print("BLUE: " + str(self.areas[2]))
                print(self.data.ranges[50])
                self.areas[0] *= 1.3
                if self.areas[0] > 15000:
                        self.areas[0] -= 14000
                if self.areas[0] > 40000:
                        self.areas[0] -= 35000
                self.areas[1] *= 2
		if self.areas[1] > self.areas[0] and self.areas[1] > 900:
                        print("yellow")
                        self.cmd.drive.speed = 2
			error = 360 - self.centers[1][0]
                        self.cmd.drive.steering_angle = error * (0.000009 * self.centers[COLORS.index("y")][1] + KP)
                elif self.areas[2] > self.areas[1] and self.areas[2] > 500:
                        print("blue")
                        self.areas[0] = 0
                        self.cmd.drive.speed= 2
                        error = 360 - self.centers[2][0]
                        self.cmd.drive.steering_angle = error * (0.000005 * self.centers[COLORS.index("b")][1] + KP)
                elif self.areas[0] > self.areas[1] and self.areas[0] > 1000:
                        print("red")
                        self.cmd.drive.speed = 2
			error = 360 - self.centers[0][0]
                        self.cmd.drive.steering_angle = error * (0.000005 * self.centers[COLORS.index("r")][1] + KP)
                else:
                        print("sleeping")
                        time.sleep(0.95)
                        self.cmd.drive.speed = 0
                        print("not doing anythin")


def main():
	global AUTONOMOUS_MODE
	try:
		ic = driveStop()
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			if AUTONOMOUS_MODE:
				ic.pub.publish(ic.cmd)

	except rospy.ROSInterruptException:
		exit()


if __name__ == "__main__":
	main()
