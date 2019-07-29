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
       self.cmd.drive.speed = 100
       self.cmd.drive.steering_angle = self.angle
       #publishes the command
       self.drive_pub.publish(self.cmd)
   def find_wall(self):
   # if lidar data has not been received, do nothing
	e1 = self.data.ranges[99] - self.DESIRED_DISTANCE
	e2 = self.data.ranges[75] - self.DESIRED_DISTANCE
	if e1 > 0 and e2 > 0:
		tempAngle = 1
	elif e1 > 0 and e2 < 0:
		tempAngle = -1
	elif e1 < 0 and e2 > 0:
		tempAngle = 1
	elif e1 < 0 and e2 < 0:
		tempAngle = -1
	if self.data.ranges[50] < 1.41 * self.DESIRED_DISTANCE:
		tempAngle = -1
	if self.data.ranges[50] < .1:
		tempAngle = .4
   	return tempAngle
if __name__ == "__main__":
   rospy.init_node('wall_follower')
   wall_follower = WallFollower()
rospy.spin()
