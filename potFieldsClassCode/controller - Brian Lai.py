#!/usr/bin/env python2

import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Racecar:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'

    def __init__(self):
        self.scan = None
        
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, callback=self.scan_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        
        self.cartPoints = None
        self.finalVector = [0.5, 0]

    def scan_callback(self, msg):
        # Checks LIDAR
        self.scan = msg.ranges
        self.cartPoints = [None for x in range(len(self.scan))]
        self.convertPoints(self.scan)
        self.finalVector = self.calcFinalVector()
        self.drive()

    def convertPoints(self, lidarPoints):
        arr = []
        for i in range(len(lidarPoints)):
            arr.append(10.0/lidarPoints[i])
        self.cartPoints = list((arr[i] * math.cos(math.radians(i * 270 / (len(arr)-1) - 45)), arr[i] * math.sin(math.radians(i * 270 / (len(arr) -1) - 45))) for i in range(len(arr))) 

    def calcFinalVector(self):
        x , y= 0, 50
        length = len(self.cartPoints)
        for (i, j) in self.cartPoints:
            x -= i 
            y -= j
        # angle = -1 if x > 0 else 1
        width = self.cartPoints[int(5.0 * length / 6)] + self.cartPoints[int(length / 6.0)]
        angle = 0.2 if width < 1 else 1.0
        angle = angle * (-x / abs(x))
        
        # angle = (math.atan2(abs(y), x) - math.radians(90)) / math.pi
        return [15.0 / (max([j for (i, j) in self.cartPoints[int((5.0/12) * (len(self.cartPoints)-1)):int((7.0/12) * (len(self.cartPoints)-1))]])), angle]
    

    def drive(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.finalVector[0]
        msg.drive.steering_angle = self.finalVector[1]
        self.pub_drive.publish(msg)

    def stop(self):
        self.drive(0,0)

    def scan(self):
        return self.scan

rospy.init_node('controller')
rate = rospy.Rate(60)
rc = Racecar()


while not rospy.is_shutdown():
    # TODO implement controller logic here
    #rc.drive(0,0)
    rate.sleep()
