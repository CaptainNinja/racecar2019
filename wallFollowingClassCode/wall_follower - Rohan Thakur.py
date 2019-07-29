#!/usr/bin/env python
import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    #SIDE = rospy.get_param("wall_follower/side")
    SIDE = -1
    #VELOCITY = rospy.get_param("wall_follower/velocity")
    VELOCITY = 0.5
    #DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    DESIRED_DISTANCE = 1.0

    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.data = None
        self.angle = 0
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)


    def scan(self, data):
        #stores the lidar data so you can work with it
        self.data = data
        #calls function that controls driving
        self.drive()


    def drive(self):
        """controls driving"""
        #gets the angle required
        self.angle = self.find_wall()
        #sets speed and driving angle
        self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.steering_angle = self.angle
        #publishes the command
        self.drive_pub.publish(self.cmd)


    def find_wall(self):
        # if lidar data has not been received, do nothing
        if self.data == None:
            return 0
        ## TO DO: Find Alg for Wall Following ##

        """Lidar data is now stored in self.data, which can be accessed
        using self.data.ranges (in simulation, returns an array).
        Lidar data at an index is the distance to the nearest detected object
        self.data.ranges[0] gives the leftmost lidar point
        self.data.ranges[99] gives the rightmost lidar point
        self.data.ranges[50] gives the forward lidar point
        """
        TURN_ANGLE = 15
        CORNER_CONSTANT = 2
        LARGE_DIST = self.DESIRED_DISTANCE * 3
        tempAngle = 0
        numPoints = int((self.data.angle_max - self.data.angle_min) / float(self.data.angle_increment))
        if self.SIDE == 1:
            distances = [self.data.ranges[i] for i in range(int(numPoints / 2), numPoints) if self.data.ranges[i] >= 0]
            dist1 = distances[int(float(35) / 50 * len(distances))]
            dist2 = distances[int(float(25) / 50 * len(distances))]
            error1 = self.DESIRED_DISTANCE - dist1
            error2 = self.DESIRED_DISTANCE - dist2

            if len(distances) < int(numPoints / 2) - 1:
                tempAngle = TURN_ANGLE * math.pi/180 * 1/5
            elif distances[0] < CORNER_CONSTANT*self.DESIRED_DISTANCE:
                tempAngle = -1 * TURN_ANGLE * math.pi/180
            elif dist1 > LARGE_DIST and dist2 > LARGE_DIST:
                tempAngle = 0
            elif error1 > 0 and error2 > 0:
                tempAngle = -1 * TURN_ANGLE * math.pi/180
            elif error1 > 0 and error2 < 0:
                tempAngle = TURN_ANGLE * math.pi/180 * 1/2
            elif error1 < 0 and error2 > 0:
                tempAngle = -1 * TURN_ANGLE * math.pi/180 * 1/2
            elif error1 < 0 and error2 < 0:
                tempAngle = TURN_ANGLE * math.pi/180

        if self.SIDE == -1:
            distances = [self.data.ranges[i] for i in range(int(numPoints / 2)) if self.data.ranges[i] >= 0]
            dist1 = distances[int(float(25) / 50 * len(distances))]
            dist2 = distances[int(float(35) / 50 * len(distances))]
            error1 = self.DESIRED_DISTANCE - dist1
            error2 = self.DESIRED_DISTANCE - dist2
            if len(distances) < int(numPoints / 2) - 1:
                tempAngle = -1 * TURN_ANGLE * math.pi/180 * 1/4
            elif distances[len(distances) - 1] < CORNER_CONSTANT*self.DESIRED_DISTANCE:
                tempAngle = TURN_ANGLE * math.pi/180
            elif dist1 > LARGE_DIST and dist2 > LARGE_DIST:
                tempAngle = 0
            elif error1 > 0 and error2 > 0:
                tempAngle = TURN_ANGLE * math.pi/180
            elif error1 > 0 and error2 < 0:
                tempAngle = -1 * TURN_ANGLE * math.pi/180 * 1/2
            elif error1 < 0 and error2 > 0:
                tempAngle = TURN_ANGLE * math.pi/180 * 1/2
            elif error1 < 0 and error2 < 0:
                tempAngle = -1 * TURN_ANGLE * math.pi/180
        #returns the output of your alg, the new angle to drive in
        return tempAngle


    def polarToCartesian(self, r, theta):
        x = r*math.cos(theta)
        y = r*math.sin(theta)
        #x = r*math.sin(theta)
        #y = r*math.cos(theta)
        return np.array([x, y])


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()