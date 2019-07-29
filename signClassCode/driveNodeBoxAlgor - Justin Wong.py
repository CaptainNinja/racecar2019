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
from turnRectStarterBoxAlgor import sift_det

#initalize global variables

AUTONOMOUS_MODE = True
STOPPING_AREA = 12000
CENTER_THRESHOLD = 15
TURN_ANGLE = 15

#What should your drive topic be?
DRIVE_TOPIC = '/drive'
IMAGE_TOPIC = '/zed/zed_node/testingImage'


class driveStop(object):
    """class that will help the robot drive and stop at certain conditions
    """
    def __init__(self):
        """initalize the node"""

        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.zed_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 1)
        self.bridge = CvBridge()
        self.imageWithBoxes = None
        rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

        """ initialize the box dimensions"""
        #self.box_size = color_segmentation.cd_color_segmentation()
        xSize = 0
        ySize = 0

        """driving code"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 2
        self.cmd.drive.steering_angle = 0
    
        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image = False)

        self.min_value=0

    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""

        upperLeft = self.flag_box[0]
        bottomRight = self.flag_box[1]
        self.xSize = np.abs(bottomRight[0] - upperLeft[0])
        self.ySize = np.abs(upperLeft[1] - bottomRight[1])


    def driveStop_car_callback(self,data):
        """laser scan callback function"""
        #checks if the image is valid first
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

    def detectCone(self):
        print('CONE: ' + str(self.xSize * self.ySize))
       
        if self.xSize * self.ySize >= STOPPING_AREA:
            print('CONE: STOP')
            self.cmd.drive.speed = 0
        else:
            self.cmd.drive.speed = 2
            avg = (self.flag_box[0][0] + self.flag_box[1][0]) / 2
            if avg < 360 - CENTER_THRESHOLD:
                print("CONE: turning left")
                self.cmd.drive.steering_angle = TURN_ANGLE * math.pi/180
            elif avg > 360 + CENTER_THRESHOLD:
                print("CONE: turning right")
                self.cmd.drive.steering_angle = -TURN_ANGLE * math.pi/180
            else:
                print("CONE: going straight")
                self.cmd.drive.steering_angle = 0  

    def drive(self):
        
        gBoxCoor, rBoxCoor, area, self.imageWithBoxes = sift_det("oneway.jpg", self.camera_data.cv_image)
        self.zed_pub.publish(self.bridge.cv2_to_imgmsg(self.imageWithBoxes, "bgr8"))
        #if len(gBoxCoor) == 0 or len(rBoxCoor) == 0:
        #    return
        print('redBox: ')
        print(rBoxCoor)
        print('greenBox: ')
        print(gBoxCoor)
        if len(rBoxCoor) == 2 and len(gBoxCoor) == 2:
            print('SIGN: ' + str(area))
            if area >= 6000:
                if gBoxCoor[-2][0] < rBoxCoor[-2][0]:
                    print("SIGN: turn left")
                    self.cmd.drive.steering_angle = 60 * math.pi/180
                    if self.xSize * self.ySize > 100:
                        self.detectCone()
                else:
                    print("SIGN: turn right")
                    self.cmd.drive.steering_angle = -60 * math.pi/180
                    if self.xSize * self.ySize > 100:
                        self.detectCone()
        #elif len(rBoxCoor) == 0 or len(gBoxCoor) == 0:
            #print('the lengths are 0')
            #self.detectCone()
        else:
            print('SIGN: nothing interesting is happening')
            self.cmd.drive.speed = 2
            self.cmd.drive.steering_angle = 0

            
    
           

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
