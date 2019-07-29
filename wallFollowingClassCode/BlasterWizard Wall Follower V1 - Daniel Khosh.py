#!/usr/bin/env python
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
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
        #Algorithm for driving
        #gets the angle required
        self.angle = self.find_wall()
        #sets speed and driving angle
        self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.steering_angle = self.angle
        #publishes the command
        self.drive_pub.publish(self.cmd)

    def find_wall(self):
        tempAngle = 0
        KP = 1
        KD = 2
        length = len(self.data.ranges)
        if self.SIDE == 1:
            #Prevents circles
            if self.data.ranges[length*5/6] < self.data.ranges[length/2]:
                temp = -KP*(self.DESIRED_DISTANCE - self.data.ranges[length*5/6])
            else:
                temp = KP*(self.DESIRED_DISTANCE - self.data.ranges[length*5/6])
            tempAngle = temp - KD*(3.14/4/(pow(self.data.ranges[length*5/6],2) + pow(self.data.ranges[length*2/3],2) - 2*3.14/4*self.data.ranges[length*5/6]*self.data.ranges[length*2/3])*self.data.ranges[length*5/6]-3.14/4)
        else:
            #Prevents circles
            if self.data.ranges[length/6] < self.data.ranges[length/2]:
                temp = KP*(self.DESIRED_DISTANCE - self.data.ranges[length/6])
            else:
                temp = -KP*(self.DESIRED_DISTANCE - self.data.ranges[length/6])
            tempAngle = temp + KD*(3.14/4/(pow(self.data.ranges[length/6],2) + pow(self.data.ranges[length/3],2) - 2*3.14/4*self.data.ranges[length/6]*self.data.ranges[length/3])*self.data.ranges[length/6]-3.14/4)
        
        #Assists with corners
        if self.data.ranges[length/2] < self.DESIRED_DISTANCE*1.5:
            self.VELOCITY = rospy.get_param("wall_follower/velocity")/2
            if sum(self.data.ranges[0:length/2]) < sum(self.data.ranges[length/2:length]):
                tempAngle = 5
            else:
                tempAngle = -5
        else:
            self.VELOCITY = rospy.get_param("wall_follower/velocity")
        """Lidar data is now stored in self.data, which can be accessed
        using self.data.ranges (in simulation, returns an array).
        Lidar data at an index is the distance to the nearest detected object
        self.data.ranges[0] gives the leftmost lidar point
        self.data.ranges[100] gives the rightmost lidar point
        self.data.ranges[50] gives the forward lidar point
        """
        # if lidar data has not been received, do nothing
        if self.data == None:
            return 0
        #returns the output of your alg, the new angle to drive in
        return tempAngle

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()