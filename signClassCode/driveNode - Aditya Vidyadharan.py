#!/usr/bin/python

#import libraries and color segmentation code
import rospy
import cv2
import time
import math
import numpy as np
from newZed import Zed_converter
#from cv_bridge import CvBridge, CvBridgeError
from turnRectStarter import sift_det
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
#initalize global variables

AUTONOMOUS_MODE = True

#What should your drive topic be?
DRIVE_TOPIC = '/drive'
SCAN_TOPIC = '/scan'

class driveStop(object):
	"""class that will help the robot drive and stop at certain conditions
	"""
        CONTROLLER_TOPIC = "/vesc/joy"
	def __init__(self):
		"""initalize the node"""
		rospy.init_node("driveStop")
		self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
		self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveStop_car_callback)
		#rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
                #self.joy_scan = rospy.Subscriber(self.CONTROLLER_TOPIC, Joy, callback = self.joy_callback)
		""" initialize the box dimensions"""

                """driving code"""
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 0
		self.cmd.drive.steering_angle = 0

		"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)
		self.min_value=0
        self.min_cone = 500
		self.timeToTurn = False
        self.timeToTurnRight = False
        #def joy_callback(self, msg):
        #    self.buttons = msg.axes
        def directionInput(self):
            if self.buttons[5] >= 0:
                return True
            if self.buttons[5] <= 0:
                return False
	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
		return abs((self.flag_box[1][0]-self.flag_box[0][0])*(self.flag_box[0][1]-self.flag_box[1][1]))
        '''
	def scan_callback(self, data):
            Checks LIDAR data
            self.data = data.ranges
        '''
	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
                self.data = data.ranges
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")

		#applies the current filter to the image and returns a bounding box
		self.flag_box = cd_color_segmentation(self.camera_data.cv_image)

		#finds the size of the box
		self.size_calc()

		if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass

	def drive(self):
                """write driving commands here! You don't need to publish the drive command,
                that is being taken care of in the main() function"""
                #print(self.directionInput())
		#if self.size_calc() < 2500: # may change
		#	self.cmd.drive.speed = 0.7
		#else:
		#	self.cmd.drive.speed = 0


		if self.data[int(round(len(self.data)/2))] <= 1.5 and self.data[int(round(len(self.data)/6))] >= 0.1 and self.data[int(round(len(self.data)/6))*5] >= 0.1:
                        if sift_det("images/oneway.jpg", self.camera_data.cv_image): #how to pass file
				self.timeToTurn = True
				self.timeToTurnRight = True
                        else:
				self.timeToTurn = True
				self.timeToTurnRight = False
		else:
			self.cmd.drive.steering_angle = 0

		if self.timeToTurn and self.size_calc() < self.min_cone: # self.size_calc() < 100
			if self.timeToTurnRight:
				self.cmd.drive.speed = 1
				self.cmd.drive.steering_angle = 1
			else:
				self.cmd.drive.speed = 1
				self.cmd.drive.steering_angle = -1
                else:
			self.timeToTurn = False

                if self.size_calc() > self.min_cone and (self.flag_box[1][0]+self.flag_box[0][0])/2 < 375: #int(round(len(self.camera_data.cv_image)/2)): # self.size_calc() > 100
                    self.cmd.drive.speed = 2
                    self.cmd.drive.steering_angle = 0.3
                    #print((self.flag_box[1][0]+self.flag_box[0][0])/2)
                elif self.size_calc() > self.min_cone and (self.flag_box[1][0]+self.flag_box[0][0])/2 > 375: #int(round(len(self.camera_data.cv_image)/2)):
                    self.cmd.drive.speed = 2
                    self.cmd.drive.steering_angle = -0.3
                    #print((self.flag_box[1][0]+self.flag_box[0][0])/2)
                elif self.size_calc() > self.min_cone and (self.flag_box[1][0] + self.flag_box[0][0])/2 == 375: #int(round(len(self.camera_data.cv_image)/2)):
                    self.cmd.drive.speed = 0
                if self.size_calc() > 500:
                    print("cone")
                if self.size_calc() < 22000:
                    self.cmd.drive.speed = 2
                else:
                    self.cmd.drive.speed = 0
                #if self.data[int(round(len(self.data)/2))] < 2:
                #    self.cmd.drive.speed = 0.2

                #self.cmd.drive.steering_angle = 0
'''
def find_sign(self):
        color_box, sign_box = sift_det("images/oneway.jpg",self.camera_data.cv_image)
        if color_box == None or sign_box == None:
            return False
        if abs(color_box[0][0]-sign_box[0][0]) < 100:
                x_diff = color_box[0][0] - sign_box[0][0]
                if x_diff < 0:
                    return True
                else:
                    return False
'''
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
