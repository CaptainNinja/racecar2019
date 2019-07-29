#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from newZed import Zed_converter
#from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
from turnRectStarter import orb_det

#initalize global variables

AUTONOMOUS_MODE = True

#What should your drive topic be?
DRIVE_TOPIC ="/drive"


class driveStop(object):
	"""class that will help the robot drive and stop at certain conditions
	"""
	def __init__(self):
		#"""initalize the node"""
		rospy.init_node("driveStop")
		self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
		self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,LaserScan,callback=self.scan_callback)
		rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
		#""" initialize the box dimensions"""
		self.flag_box = ((0,0),(0,0))
                #"""driving code"""

		self.size = 0
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 0
		self.cmd.drive.steering_angle = 0
		#"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)
		self.min_value=0

	def size_calc(self):
		#""" calculate the x and y size of the box in pixels"""

		#flag_box is a tuple of 2 tuples
		#contains coordinates of top left and bottom right
                """
		#Top Left
		TL = self.flag_box[0]

		#Bottom right
		BR = self.flag_box[1]
		#print(self.flag_box)
		self.size = (BR[0]-TL[0]) * (BR[1]-TL[1])
                """

                pix_w = self.flag_box[1][0] - self.flag_box[0][0]
                #print(pix_w)
                pix_h = self.flag_box[1][1] - self.flag_box[0][1]
                self.size = pix_w*pix_h

	def driveStop_car_callback(self,data):
		#"""laser scan callback function"""
		#checks if the image is valid first
		(self.redRectangleVertices, self.greenRectangleVertices) = orb_det("./oneway.txt",self.camera_data.cv_image)
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")

		#applies the current filter to the image and returns a bounding box
		self.flag_box = cd_color_segmentation("./oneway",self.camera_data.cv_image)
		#print("coordinates1:")
                #print(self.flag_box)
		#print(self.flag_box)
		rectPOne = (self.greenRectangleVertices[1][0],self.greenRectangleVertices[0][1])
		rectPTwo = self.greenRectangleVertices[0]
		rectPThree = (self.greenRectangleVertices[0][0],self.greenRectangleVertices[1][1])
		rectPFour = self.greenRectangleVertices[1]
		rectPTM = (((self.greenRectangleVertices[0][0]+self.greenRectangleVertices[1][0])/2),self.greenRectangleVertices[0][1])
		rectPBM = (((self.greenRectangleVertices[0][0]+self.greenRectangleVertices[1][0])/2),self.greenRectangleVertices[1][1])

		frame = self.camera_data.cv_image
		trueFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

		rectOne = trueFrame[rectPTwo[0]:rectPBM[0],rectPTwo[1]:rectPBM[1]]
		rectTwo = trueFrame[rectPTM[0]:rectPFour[0],rectPTM[1]:rectPTM[1]]

		whiteOne = np.sum(rectOne==255)
		whiteTwo = np.sum(rectTwo==255)

		if whiteOne > whiteTwo: #turn left
			self.cmd.drive.steering_angle = -60
		else: #turn right
			self.cmd.drive.steering_angle = 60


		if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass

	def drive(self):
        #"""write driving commands here! You don't need to publish the drive command,
        #that is being taken care of in the main() function"""



        	self.size_calc()
		#print(self.size)
		middle = (self.flag_box[0][0]+self.flag_box[1][0])/2
		print(middle)
		if self.size > 13000 and middle < 440 and middle > 360:
			self.cmd.drive.speed = 0
		else:
			#if (<=25):
			#else:
			self.cmd.drive.speed = 0.5


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
