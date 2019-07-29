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
from turnRectStarter import sift_det

#initalize global variables

AUTONOMOUS_MODE = True
COUNT = 0
#What should your drive topic be?
DRIVE_TOPIC = '/drive'
source = './images/oneway.jpg'
COUNT = 0

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
		self.cmd.drive.speed = 0.5
		self.cmd.drive.steering_angle = 0
                self.center1 = (309, 158)
                self.center2 = (347, 219)
                self.signTrack = True
		"""get the camera data from the class Zed_converter in Zed"""
                self.camera_data = Zed_converter(False, save_image = False)

		self.min_value=0

	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
		point1, point2 = self.flag_box
		#x1 = point1[0]
		#x2 = point2[0]
		#y1 = point1[1]
		#y2 = point2[1]
                print('point 1: ' + str(point1) + 'point 2: ' + str(point2))
                return (point1, point2) 
 #180-255, 89 197







	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")
		
		#applies the current filter to the image and returns a bounding box
		
                self.flag_box = cd_color_segmentation(self.camera_data.cv_image)
                self.direction = sift_det(self.camera_data.cv_image, source)

		#finds the size of the box
		#self.size_calc()
		print('in callback')
		if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass
	
	def drive(self):
            total = 0
            print('driving')
            ''' for i in range(20):
i                    if(self.size_calc() == None):
                        pass
                    else:
                        total += self.size_calc()
                if total/20 >= 3500:
                    print('cone in range')
                    self.cmd.drive.speed = 0
                else:
                pass'''
            print(COUNT)
            if(COUNT < 47000):
                print('in sign tracking mode')
                
                #if(self.direction == 2 and COUNT > 35000):
                if(self.direction == 1 and COUNT > 30000):
                    #to the right
                    self.cmd.drive.speed = 0.5
                    #self.cmd.drive.steering_angle = -1 * np.pi / 4.0
                    self.cmd.drive.steering_angle = -1 * np.pi / 0.5
                    print('sign to the right')
                    signTrack = False
                #elif(self.direction == 1 and COUNT > 35000):
                elif(self.direction == 2 and COUNT > 30000):
                    self.cmd.drive.speed = 0.5
                    #self.cmd.drive.steering_angle = np.pi / 4.0
                    self.cmd.drive.steering_angle = np.pi / 2.0
                    print('sign to the left')
                    signTrack = False
                    #to the left
                else:
                    self.cmd.drive.speed = 0.5
                    self.cmd.drive.steering_angle = 0
                    print('no sign')
            else:
                print('in cone tracking mode')

                currentP1, currentP2 = self.size_calc()
                area = (currentP2[0] - currentP1[0]) * (currentP2[1] - currentP1[1])
                if(currentP1[0]-self.center1[0] > 8):# and area >= 2000):
                    print('cone is to the right')
                    self.cmd.drive.steering_angle = -1 * np.pi / 3
                elif(currentP1[0] - self.center1[0] < -8):# and area >= 2000):
                    print('cone is to the left')
                    self.cmd.drive.steering_angle = np.pi / 3
                else:
                    self.cmd.drive.steering_angle = 0
                    print('cone is in the center')
                    '''if(abs(currentP1[0]-308) <= 10):
                    if(abs(currentP2[1] - 376) <= 10):
                    print('too close')
                    self.cmd.drive.speed = 0
                    else:
                    print('good range')
                    self.cmd.drive.speed = 0.5
                    '''
                print('area =' + str(area))
                if(area >= 24500):
                    self.cmd.drive.speed = 0
                else:
                    self.cmd.drive.speed = 0.5
                    #self.cmd.drive.speed = 5
                    #self.cmd.drive.speed = 3









        


def main():
	global AUTONOMOUS_MODE
        global COUNT
	try:
		ic = driveStop()
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			if AUTONOMOUS_MODE:
                                #print(ic.cmd.drive.steering_angle)
				ic.pub.publish(ic.cmd)
                                COUNT += 1
                                #print(COUNT)
			
	except rospy.ROSInterruptException:
		exit()


if __name__ == "__main__":
	main()
