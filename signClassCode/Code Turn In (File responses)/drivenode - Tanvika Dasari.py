#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from newZed import Zed_converter
import math
#from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from turnRectStarter import sift_det
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables

AUTONOMOUS_MODE = True

#What should your drive topic be?
DRIVE_TOPIC = "/drive"


class driveStop(object):
	"""class that will help the robot drive and stop at certain conditions
	"""

	def __init__(self):
		"""initalize the node"""
		rospy.init_node("driveStop")
		self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
		self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)


		#""" initialize the box dimensions"""
		self.width = 0.0
		self.height = 0.0
		self.area = 0.0
		self.screen_cent = 672/2
		self.h_middle = 0
		self.cone = False
		self.left = False
		self.right = False


    	#"""driving code"""
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 0
		self.cmd.drive.steering_angle = 0

		#"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)

		self.min_value=0


	def scan_callback(self, data):
		self.data = data.ranges

	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
		if self.flag_box != ((0,0),(0,0)):
			self.cone = True

		tlc = self.flag_box[0]
		brc = self.flag_box[1]

		self.width = brc[0] - tlc[0]
		self.height = brc[1] - tlc[1]

		self.area = self.width * self.height
		self.h_middle = (brc[0]+tlc[0])/2

		pass


	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			#print("sleeping")

		#applies the current filter to the image and returns a bounding box
		self.flag_box = cd_color_segmentation(self.camera_data.cv_image)
		self.arrow = sift_det("images/oneway.jpg", self.camera_data.cv_image) #may moved 2init
		#print(self.arrow)


		#finds the size of the box
		self.size_calc()

		if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass

	def drive(self):

		
		print(self.data[540])
		self.cmd.drive.steering_angle = 0
		if(self.arrow[0] == None or self.arrow[1] == None):
			self.cmd.drive.speed = 0.5
		else:
			self.cmd.drive.speed = 0
			print("i see it")
			diff_l = (self.arrow[0][0][0] - self.arrow[1][0][0])
			diff_r = (self.arrow[0][1][0]- self.arrow[1][1][0])
			print(diff_l,diff_r)
		return 0
		#Preportional steering controller:
		'''
		if(self.arrow[0] == None and self.left == False and self.right == False):

			print("can't see the arrow")
			self.cmd.drive.speed = 0.5

		elif((self.arrow[0] != None and self.arrow[1]  != None) ):
			
			diff_l = (self.arrow[0][0][0] - self.arrow[1][0][0])
			diff_r = (self.arrow[0][1][0]- self.arrow[1][1][0])
			
			print(diff_l,diff_r)
			difference = diff_r
			
			if difference > 0:
				self.left = True
				self.cmd.drive.steering_angle = 1
				#print('left')
			if difference < 0:
				self.right = True
				self.cmd.drive.steering_angle = -1
				#print('right')
		'''
			
				


		if self.cone == True:
			error = (self.screen_cent - self.h_middle)
			gain = 1/180
			angle = (gain*error)

			if error < 0:
				angle = angle * -1

		#print(error, self.area)

			self.cmd.drive.steering_angle = angle
			print(self.area)

		'''
		if self.h_middle < (self.screen_cent-100):
			self.cmd.drive.steering_angle = 1
		elif self.h_middle > (self.screen_cent+100):
			self.cmd.drive.steering_angle = -1
		else:
			self.cmd.drive.steering_angle = 0
		'''

		if (abs(self.area) >= 15000):
			self.cmd.drive.speed = 0
			#print('stopped')

			return 0

		self.cmd.drive.speed = 0
		pass


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
