#!/usr/bin/env python2
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Racecar:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    # SIDE = -1
    # VELOCITY = 5
    # DESIRED_DISTANCE = .5
    DIST_CONSTANT = 2.0
    CARROT_DIST = .2

    def __init__(self):
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,
                                         LaserScan,
                                         callback=self.scan_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC,
                                         AckermannDriveStamped,
                                         queue_size=1)
        self.data = None

    def scan_callback(self, msg):
        self.data = msg

    def drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)

    def stop(self):
        self.drive(0,0)

    def scan(self):
        return self.scan

    def convertPoints(self, points):
        '''Convert all current LIDAR data to cartesian coordinates that have been inverted'''
        convertedPoints = np.zeros((int(len(points) * float(4)/3) + 1, 2))
        for i in range(len(points)):
            r = -1 * self.DIST_CONSTANT / points[i]
            theta = (self.data.angle_min + self.data.angle_increment*i)
            # if -1*math.pi/2 < theta < math.pi/2:
            convertedPoints[i, 0] = r*math.cos(theta)
            convertedPoints[i, 1] = r*math.sin(theta)
            # if -0.05 < theta < 0.05:
            #     print(convertedPoints[i])
        # convertedPoints[len(points):(len(points) + len(convertedPoints)) / 2] = np.array([convertedPoints[i, 0], 0])
        # convertedPoints[(len(points) + len(convertedPoints)) / 2:] = np.array([convertedPoints[0, 0], 0])

        # print('input: ' + str(points))
        # print('output: ' + str(convertedPoints))
        return convertedPoints

    def calcFinalVector(self, points):
        '''Calculate the final driving speed and angle'''
        sumArr = np.sum(points, axis=0)
        x = sumArr[0]
        x += self.DIST_CONSTANT / self.CARROT_DIST
        # if x > 0:
        #     x += self.OFFSET
        # else:
        #     x -= self.OFFSET
        y = sumArr[1]
        # y += self.DIST_CONSTANT / self.CARROT_DIST

        # if y > 0:
        #     y += self.OFFSET
        # else:
        #     y -= self.OFFSET
        mag = math.sqrt(x**2 + y**2)
        # if y < 0:
        #     mag *= -1

        theta = math.atan2(y, x)
        if theta > 1:
            theta = 1
        elif theta < -1:
            theta = -1
        return (mag, theta*0.4)

rospy.init_node('controller')
rate = rospy.Rate(60)
rc = Racecar()

while not rospy.is_shutdown():
    # TODO implement controller logic here
    # if lidar data has not been received, do nothing
    if rc.data == None:
        continue

    cartPoints = rc.convertPoints(rc.data.ranges)
    finalVector = rc.calcFinalVector(cartPoints)
    # print('final vector: ' + str(self.finalVector))

    

    rc.drive(5, finalVector[1])
    rate.sleep()
