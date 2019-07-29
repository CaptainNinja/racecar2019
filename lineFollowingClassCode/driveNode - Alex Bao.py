#usr/bin/env python


#import libraries and color segmentation code
import rospy
import cv2
import time
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
		self.flag_box = ((0,0),(0,0))
		self.flag_center = (0,0)
		self.flag_size = 0
                
                self.yellow_box = ((0,0),(0,0))
                self.yellow_center = (0,0)
                self.yellow_size = 0
                
                self.blue_box = ((0,0),(0,0))
                self.blue_center = (0,0)
                self.blue_center = 0
                
                self.white_box = ((0,0),(0,0))
                self.white_center = (0,0)
                self.white_center = 0

                """driving stuff"""
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 0
		self.cmd.drive.steering_angle = 0

                self.seconds = time.time()
                self.elaspedTime = 0
                self.count = 0 	
		"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)
                self.imagePush = None
                
                self.bridge = CvBridge()
		self.min_value=0

	def size_calc(self,box):
		""" calculate the x and y size of the box in pixels"""
		pix_width = box[1][0] - box[0][0]
		pix_height = box[1][1] - box[0][1] 
		self.box_size = pix_width*pix_height

	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")

		#applies the current filter to the image and stores the image in imagePush
                self.flag_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image[240:500][0:25],np.array([0,60,147]),np.array([12,254,255]))
                self.yellow_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image[220:520][0:100],np.array([27,80,150]),np.array([40,255,255]))
                self.blue_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image[220:520][0:40],np.array([105,90,130]),np.array([115,255,255]))
                #self.white_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image[200:500][0:40],np.array([0,0,100]),np.array([255,10,255]))
                '''
                self.grayScale = cv2.cvtColor(self.camera_data.cv_image,cv2.COLOR_BGR2GRAY)
                self.cannyEdge = cv2.Canny(self.grayScale,100,200)
                lines = cv2.HoughLinesP(self.cannyEdge,1,np.pi/180,100,100,10)
                for x1,y1,x2,y2 in lines[0]:
                    a = np.cos(element[0][1])
                    b = np.sin(element[0][1])
                    x0 = a*element[0][0]
                    y0 = b*element[0][0]
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    cv2.line(self.camera_data.cv_image,(x1,y1),(x2,y2),(0,255,0),2)
                '''
                #finds the size of the box
		self.size_calc(self.flag_box)
		
                #outputs the image to the IMAGE_TOPIC
                try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
                except CvBridgeError as e:
                        print("Error bridging Zed image", e)
		
                if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass
	
	def turn(self,box):
            if box[1][0] == 0 and box[0][0] == 0:
                diffAngle = 0
            else:
                diffAngle = abs((box[1][0]+box[0][0])/2-370)/370.0
            if (box[1][0]+box[0][0])/2 < 370:
                self.cmd.drive.steering_angle = diffAngle
            elif (box[1][0]+box[0][0])/2 > 370:
                self.cmd.drive.steering_angle = -diffAngle
            else:
                self.cmd.drive.steering_angle = 0
            #print(str(self.flag_box[1][0]) +  "," + str(self.flag_box[0][0]))

        def drive(self):
            self.cmd.drive.speed = 5
            self.turn(self.flag_box)
            if self.blue_box[1][0] != 0 and self.blue_box[0][0] != 0:
                self.turn(self.blue_box)
            if self.yellow_box[1][0] != 0 and self.yellow_box[0][0] != 0: 
                self.turn(self.yellow_box)
            self.elaspedTime = time.time() - self.seconds
            #print(self.elaspedTime)
            if self.yellow_box[1][0] == 0 and self.blue_box[0][0] == 0 and self.flag_box[1][0] == 0:
                self.count += 1
            if self.count > 42:
                self.cmd.drive.speed = 0

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
