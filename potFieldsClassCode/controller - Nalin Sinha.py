#!/usr/bin/env python2
from math import *
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

k = 1.0
carCharge = 1.0
jetCharge = 0.8
wallCharge = .6
lidarPoints = 100

initSpeed = 0.2
steeringFactor = 2.3
speedFactor = 0.1
class Racecar:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'

    def __init__(self):
        self.angle = 0
        self.data = None
        self.current_speed = initSpeed
        #x (steering)
        self.iHat = []
        #y (speed)
        self.jHat = []
        #[speed, angle]
        self.finalVector = [0, 0.5]
        self.cmd = AckermannDriveStamped()
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,
                                         LaserScan,
                                         callback=self.scan_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC,
                                         AckermannDriveStamped,
                                         queue_size=1)



    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.iHat = []
        self.jHat = []
        for i, item in enumerate(self.data):
            i+=1
            angle = float(i)/lidarPoints * (3*pi/2) - (pi/4)
            maga = -k * (carCharge * wallCharge)/(item ** 2)
            self.iHat.append(-maga * cos(angle))
            self.jHat.append(maga * sin(angle))

        self.finalVector = [sum(self.iHat), sum(self.jHat) + (k * jetCharge)]
        #print(self.finalVector)
        self.drive_callback()

    def drive_callback(self):
        self.cmd.drive.speed += speedFactor * self.finalVector[1]
        #self.cmd.drive.speed = 0
        self.angle = steeringFactor * self.finalVector[0]
        self.cmd.drive.steering_angle = self.angle
        #print(self.angle)
        '''Publishes drive commands'''
        self.pub_drive.publish(self.cmd)

        #make sure to publish cmd here

    def stop(self):
        self.drive(0,0)

    def scan(self):
        return self.scan

rospy.init_node('controller')
rate = rospy.Rate(60)
rc = Racecar()

while not rospy.is_shutdown():
    # TODO implement controller logic here
    rospy.spin()
