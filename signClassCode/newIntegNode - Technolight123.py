#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
import argparse
import sys, math, random, copy
import rospy, copy, time
from newZed import Zed_converter
#from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables

AUTONOMOUS_MODE = True

#What should your drive topic be?
DRIVE_TOPIC = "/drive"
rospy.init_node("driveStop")
args=1

class driveStop(object):
        """class that will help the robot drive and stop at certain conditions """
        def __init__(self):
                """initalize the node"""
                self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
                self.sub= rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
                ##initialize the box dimensions
                ##driving code
        		
                self.cmd = AckermannDriveStamped()
                self.cmd.drive.speed = 0
                self.cmd.drive.steering_angle = 0
                self.x_mid= None
                self.y_bound= None
                self.no_drive=False
                self.phase=0
                self.results=[None,None] ##first pos is deviation of x box middle from center of image
                ##second pos is how close y box pos is bottom
                """get the camera data from the class Zed_converter in Zed"""
                self.camera_data = Zed_converter(False, save_image = False)

	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
		if self.flag_box[0][0]+self.flag_box[1][0]<=0:
                        self.results=None
                else:
                        image_props=self.camera_data.cv_image.shape
                        self.y_bound=int(image_props[0])
                        self.x_mid=int(image_props[1]/float(2))
                        box_x_mid=self.flag_box[0][0]+float(self.flag_box[1][0]-self.flag_box[0][0])/2
                        box_y_pos=self.flag_box[1][1]
                        self.results=[box_x_mid-self.x_mid,self.y_bound-box_y_pos]
                        ##print (self.results)
                        
	def driveStop_car_callback(self, data):
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
	
	def drive(self):
                ##write driving commands here! You don't need to publish the drive command,that is being taken care of in the main() function
                ##phase 0
                if self.phase==0:
                        self.sift_det(self.camera_data.cv_image)
                ##phase 1
                if self.phase==1:
                        if self.results==None:
                                self.cmd.drive.speed=0.2
                                self.cmd.drive.steering_angle=0
                                print("autoing")
                        else:
                                print(self.results[1])
                                if self.results[1]<=167:
                                        self.cmd.drive.speed=0
                                        self.no_drive=True
                                elif self.no_drive==False:
                                        self.cmd.drive.speed=1
                                else:self.cmd.drive.speed=0
                                if self.results[0]>0:
                                        self.cmd.drive.steering_angle=-1*self.results[0]/self.x_mid
                                        ##going to the right
                                elif self.results[0]<0:
                                        self.cmd.drive.steering_angle=-1*self.results[0]/self.x_mid
                                        ##going to the left
                                else:
                                        self.cmd.drive.steering_angle=0
                if (self.sift_det(self.camera_data.cv_image)):
                        self.cmd.drive.steering_angle=-1
                        self.cmd.drive.speed=1
                else:
                        self.cmd.drive.steering_angle=1
                        self.cmd.drive.speed=1
        def sift_det(self, image):
                self.drive_pub = rospy.Publisher(DRIVE_TOPIC,AckermannDriveStamped, queue_size=1)
                # TODO: # Sets up match count between images for sensitivity of detection- choose your value!
                MIN_MATCH_COUNT = 10
                sift=None
                # If VideoCapture(feed) doesn't work, manually try -1, 0, 1, 2, 3 (if none of those work, 
                # the webcam's not supported!)
                cam = self.camera_data.cv_image

                # Reads in the image
                img1 = image                    

                # Labels the image as the name passed in    
                if self.camera_data.cv_image is not None:
                        label = "label"
                else:
                    # Takes the name of the image as the name
                        if image[:2] == "./":
                                label = label = (image.split("/"))[2]
                        else:
                                label = image[2:-4]

                ################################################### Set up Feature Detection

                # Create a the SIFT Detector Object
                orb = cv2.ORB_create()

                # Compute keypoints
                kp, des = orb.detectAndCompute(img1, None)

                FLANN_INDEX_KDTREE = 0
                # Option of changing 'trees' and 'checks' values for different levels of accuracy
                index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)    
                search_params = dict(checks = 50)                    

                # Fast Library for Approximate Nearest Neighbor Algorithm
                # Creates FLANN object for use below
                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = False)


                frame = self.camera_data.cv_image

                ##if frame is None:             # Did we get an image at all?
                        ##continue

                ################################################### Shape Computation

                # TODO:
                # What color space does OpenCV read images in, and what color space do we want process?
                # Check out cvtColor! <https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html>
                # Read in the image from the camera 
                img = cv2. cvtColor(frame, cv2.COLOR_RGB2HSV)
                
                # TODO: 
                # Set up your HSV threshold bounds - [Hue, Saturation, Value]
                lower = np.array([0, 0, 0], dtype = "uint8")
                upper = np.array([200, 200, 30], dtype = "uint8")     

                # TODO: 
                # Check inRange() <https://docs.opencv.org/3.0-beta/modules/core/doc/operations_on_arrays.html?highlight=inrange#invert>
                # Create mask for image with overlapping values
                mask = cv2.inRange(img, lower, upper)

                # TODO:
                # What parameters work best for thresholding? <https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html?highlight=adaptivethreshold>
                imgThresh = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 3, 1)
                
                # TODO:
                # This is OpenCV's call to find all of the contours
                # Experiment with different algorithms (cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE) 
                # in the parameters of cv2.findContours!
                # <https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findContours>
                # The underscores represent Python's method of unpacking return values, but not using them
                _, contours, _ = cv2.findContours(imgThresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

                # Optional TODO:
                # Optional processing of contours - do we want to remove all non-rectangle shapes from contours?
                # Read the documentation on approxPolyDP <https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html>

                # TODO:
                # Orders contours - but by what metric? Check the "key" options <https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html>
                # Ex. key = cv2.contourVectorLen() (Would order the contours by vector length - not an actual function, but this is how you would set the "key")
                # Python's "sorted" function applies a "key" set lambda function to each element within an array, this is not a traditional dictionary "key"
                contours = sorted(contours, key = cv2.contourArea, reverse = True)[1:]   # Removes contouring of display window

                if len(contours) != 0:
                        # TODO:  
                        # Draws the max of the contours arrays with respect to the "key" chosen above
                        contours_max = max(contours, key = cv2.contourArea) 

                        # Find bounding box coordinates
                        rect = cv2.boundingRect(contours_max)
                        x, y, w, h = rect

                        # TODO:
                        # Calculates area of detection - what detection area should be the lower bound?
                        print(w*h)
                        if w*h > 10000:
                                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4)
                        else:
                                self.cmd.drive.speed=1

                    
                    

                    
                ################################################### Feature Detection
                # NOTE: NO CODE TO FILL IN HERE - BUT FEEL FREE TO CHANGE THE PARAMETERS

                kp_s, des_s = orb.detectAndCompute(frame, None)

                if(len(kp) >= 2 and len(kp_s) >= 2): 
                        # Uses the FLANN algorithm to search for nearest neighbors between elements of two images
                        # Faster than the BFMatcher for larger datasets
                        matches = bf.knnMatch(des, des_s, k=2)

                ##if des_s is None and len(matches) == 0:
                    ##continue

                # Store all the good matches (based off Lowe's ratio test)
                good = []
                for k, pair in enumerate(matches):
                    try:
                        (m, n) = pair
                        if m.distance < 0.75 * n.distance:
                            good.append(m)
                    except ValueError:
                        pass

                # When there are enough matches, we convert the keypoints to floats in order to draw them later
                #print(len(good))
                if len(good) >= MIN_MATCH_COUNT:
                    try:
                        src_pts = np.float32([ kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                        dst_pts = np.float32([ kp_s[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                    except IndexError:
                        pass

                    # Homography adds a degree of rotation/translation invariance by mapping the transformation 
                    # of points between two images
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    matchesMask = mask.ravel().tolist()

                    h, w = (img1.shape[:2])
                    pts = np.float32([ [0,0],[0,h-2],[w-2,h-2],[w-2,0] ]).reshape(-1,1,2)

                    if M is not None:
                        dst = cv2.perspectiveTransform(pts, M)
                        intDst = np.int32(dst)

                        # Draws a bounding box around the area of most matched points
                        cv2.rectangle(frame, (intDst[0][0][0], intDst[0][0][1]), (intDst[2][0][0], intDst[2][0][1]), (0, 0, 255), 4, cv2.LINE_AA, 0)
                        cv2.putText(frame, label, (dst[0][0][0], dst[0][0][1]) , cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), lineType = cv2.LINE_AA )
                        print("x diff is " + str(abs(x-intDst[0][0][0])))
                        if abs(x-intDst[0][0][0]) > 100:
                            print("not same")
                            return False
                        else:
                           print("same")
                           return True

                    else:
                        matchesMask = None

                else:
                    matchesMask = None

                draw_params = dict(matchColor = (0, 255, 0), # Draw matches in green color
                                   singlePointColor = None,
                                   matchesMask = matchesMask, # Draw only inliers
                                   flags = 2)

                try:
                    # Option of slicing the 'good' list to display a certain number of matches (ex. good[:6])
                    # Take out draw_params if we do not want to draw matches
                    frame = cv2.drawMatches(img1, kp, frame, kp_s, good, None, **draw_params) 
                except cv2.error:
                    pass

            
        def wack(self, image):
                self.phase=1
                if(self.sift_det(image)):
                        self.cmd.drive.speed = 10
                        self.cmd.drive.steering_angle = 1
                        self.drive_pub.publish(self.cmd)
                else:
                        self.cmd.drive.speed = 10
                        self.cmd.drive.steering_angle = -1
                        self.drive_pub.publish(self.cmd)
                

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
