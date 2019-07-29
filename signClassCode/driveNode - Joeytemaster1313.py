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
import math
#from  potential-fields import 
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
        rospy.Subscriber("/scan", LaserScan, self.driveStop_car_callback)

        """ initialize the box dimensions"""
        self.box_width = 0
        self.box_height = 0
        
        "Initialize drive state"
        self.state = "init"
        self.boxes = 0

        """driving code"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
    
        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image = False)

        self.min_value=0
        
        self.size = 0

    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""
        self.box_dimensions = self.flag_box #self.flag_box is nested tuple ((x1, y1),(x2,y2))
        self.box_width = abs(self.box_dimensions[1][0] - self.box_dimensions[0][0])
        self.box_height = abs(self.box_dimensions[1][1] - self.box_dimensions[0][1])
        self.size = self.box_width * self.box_height
        return self.size

    def center(self):
        return (self.flag_box[1][0]+self.flag_box[0][0])/2





    def driveStop_car_callback(self,data):
        """laser scan callback function"""
        #checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        
        #applies the current filter to the image and returns a bounding box
        self.flag_box = cd_color_segmentation(self.camera_data.cv_image)
        #print(self.flag_box)
        self.boxes = sift_det('./images/oneway.jpg',self.camera_data.cv_image)

        #finds the size of the box
        self.size_calc()
        
        if AUTONOMOUS_MODE:
            self.drive()
        else:
            pass
    
    def sum(self, m):
        return m[0]+m[1]+m[2]

    def drive(self):
        """write driving commands here! You don't need to publish the drive command,
        that is being taken care of in the main() function"""
        """print("Width: " + str(self.box_width) + " Height: " + str(self.box_height))
        print("Area: " + str(self.size))
        print self.center()"""
        print("State: " + self.state)
        if self.state is "init":
            self.cmd.drive.speed = 1
            self.box_area()
        elif self.state is "analyze_sign":
            if type(self.boxes[0][0]) is tuple:
                self.turn()
            else:
                self.state  = "init"
        elif self.state is "turn_right": 
            self.parking_right()
        elif self.state is "turn_left": 
            self.parking_left()
        """integrate potential fields stuff"""
        """if self.analyze_sign() == "right":
            self.cmd.drive.steering_angle = -math.pi/4
            time.sleep(0.5)
            if self.parking.centered():
                self.state is "park"
            else:
                PotentialFields()
        elif self.analyze_sign() == "left":
            self.cmd.drive.steering_angle = math.pi/4
            time.sleep(0.5)
            if self.parking.centered():
                self.state is "park"
            else:
                PotentialFields() """
        
    def turn(self):
        c3 = self.boxes[1][1][0] - self.boxes[1][0][0]
        c1 = abs(self.boxes[1][1][0] - self.boxes[1][0][0])
        c2 = abs(self.boxes[1][1][1] - self.boxes[1][0][1])/5
        rects = self.boxes[1]
        g = self.camera_data.cv_image
        if(rects != None ):
            left = rects[0][0]
            top = rects[0][1]
            right = rects[1][0]
            bottom = rects[1][1]

            nx = (left+right)/2
            ny = (top+bottom)/2

            sum1 = 0
            for i in range(c1/2):
                for j in range(c2):
                        sum1 += self.sum(g[i][j-1])

            sum2 = 0
            
            for i in range(c1/2):
                for j in range(c2):
                    sum2 += self.sum(g[i+c1/2 - 1][j-1])
            print sum1, " ", sum2

            if sum1 is 0 and sum2 is 0:
                self.state = "init"
                return 0
           
            if sum1 - sum2 > 0:
                
                self.state = "turn_left"
            else:
                self.state = "turn_right"
            
            
    def box_area(self):
        #boxes = (sift_det('~/racecar_ws/src/scripts/day2Code/images/oneway.jpg',self.camera_data.cv_image))[0]
        #a1 = (self.flag_box[0][0] - self.flag_box[0][1]) * (self.flag_box[0][0] - self.flag_box[1][0])
        
        if type(self.boxes[0][0]) is not tuple:
            a2 = 0
        else:
            a2 = abs(self.boxes[1][0][0] - self.boxes[1][0][1]) * abs(self.boxes[1][0][0] - self.boxes[1][1][0])
        
        if a2 < 2000:
            self.state = "init"
        else:
            self.state = "analyze_sign"
            
    def parking_left(self):
        if self.size is not 0:
            if self.size > 8000:
                if(self.centered()):
                    self.cmd.drive.speed = 0
                    self.cmd.drive.steering_angle = 0
                else:
                    self.cmd.drive.speed = -1
                    self.cmd.drive.steering_angle = 0
            else:
                self.cmd.drive.speed = 1
                self.cmd.drive.steering_angle = (336-self.center())/300.0
        else:
            self.cmd.drive.speed = -1
            self.cmd.drive.steering_angle = -1   

    def centered(self):  #Checks if the cone is centered or not
            box_pixels = []
            centerX = self.center()
            for x in range(302,370):
                box_pixels.append(x)
            if centerX in box_pixels:
                return True
            else:
                return False     
    def parking_right(self):
        
        if self.size is not 0:
            if self.size > 8000:
                if(self.centered()):
                    self.cmd.drive.speed = 0
                    self.cmd.drive.steering_angle = 0
                else:
                    self.cmd.drive.speed = -1
                    self.cmd.drive.steering_angle = 0
            else:
                self.cmd.drive.speed = 1
                self.cmd.drive.steering_angle = (336-self.center())/300.0
        else:
            self.cmd.drive.speed = -1
            self.cmd.drive.steering_angle = 1        #print("Speed: " + str(self.cmd.drive.speed) + "Angle: " + str(self.cmd.drive.steering_angle))
    
    def analyze_sign(self):
        #analyzes sign
        #boxes= sift_det('~/racecar_ws/src/scripts/day2Code/images/oneway.jpg',self.camera_data.cv_image)
        boxes = self.boxes
        if type(boxes[0][0]) is tuple:
            green_box_x = boxes[0][0][0]
            red_box_x = boxes[1][0][0]
            difference = abs(green_box_x - red_box_x)
            print("difference is: " + str(difference))
            if difference >= 300:
                """self.cmd.drive.speed = 4
                self.cmd.drive.steering_angle = math.pi/4"""
                return "left"
            elif difference > 0 and difference < 300:
                """self.cmd.drive.speed = 4
                self.cmd.drive.steering_angle = -math.pi/4"""
                return "right"
            else:
                self.state = "park"
        else: 
            green_box_x = boxes[0][0]
            red_box_x = boxes[1][0]
            difference = abs(green_box_x - red_box_x)
            print("difference is: " + str(difference))
            if difference >= 300:
                self.cmd.drive.speed = 4
                self.cmd.drive.steering_angle = math.pi/4
            elif difference > 0 and difference < 300:
                self.cmd.drive.speed = 4
                self.cmd.drive.steering_angle = -math.pi/4
            else:
                self.state = "park"
    
   
    


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