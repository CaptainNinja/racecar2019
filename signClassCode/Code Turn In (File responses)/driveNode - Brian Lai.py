#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import math
import numpy as np
from turnRectStarter import sift_det 
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
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
		rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

		""" initialize the box dimensions"""
        
                """driving code"""
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 1
		self.cmd.drive.steering_angle = 0
	
		"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)

                # Init Vars
		self.min_value=0
                self.left = 0
                self.right = 0

	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
                x = self.flag_box[1][0] - self.flag_box[0][0]
                y = self.flag_box[1][1] - self.flag_box[0][1]  
                #print x * y
                return x * y

	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")
		
		#applies the current filter to the image and returns a bounding box
		self.flag_box = cd_color_segmentation(self.camera_data.cv_image)
                self.sign_box = sift_det("/home/racecar/racecar_ws/src/scripts/images/oneway.jpg", self.camera_data.cv_image)
		#finds the size of the box
		self.size_calc()
		
		if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass
	
	def drive(self):
            """write driving commands here! You don't need to publish the drive command,
            that is being taken care of in the main() function"""
            print self.sign_box
            print self.flag_box
            if self.sign_box[0][0][0] != 0: # Signs
                #if (self.sign_box[0][0][0] - self.sign_box[1][0][0]) > 0:
                if (self.sign_box[0][0][0] + self.sign_box[0][1][0]) / 2 - (self.sign_box[1][0][0] + self.sign_box[1][1][0]) / 2 > 10:
                    self.left += 1
                else:
                    self.right += 1
                self.cmd.drive.steering_angle = (self.left - self.right)/2
            elif self.flag_box[0][0] != 0: # Cone
                print self.size_calc()
                if self.size_calc() < 12000:
                    self.cmd.drive.speed = 50/math.sqrt(self.size_calc())
                    self.cmd.drive.steering_angle = (320 - (self.flag_box[1][0] + self.flag_box[0][0])/2+30)/300.0

                else:
                    self.cmd.drive.speed = 0
                    self.cmd.drive.steering_angle = 0.01
            else: # Drive (no bias)
	        self.cmd.drive.speed = 100
	        self.cmd.drive.steering_angle = (self.left - self.right)/2
            print "Left: " + str(self.left) + "Right: " + str(self.right) + "\n"
            print self.cmd.drive.steering_angle
            print self.cmd.drive.speed
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
