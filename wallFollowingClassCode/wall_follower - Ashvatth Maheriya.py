#!/usr/bin/env python2

import numpy as np

import rospy
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
      # TODO:
      # Initialize your publishers and
      # subscribers here
      self.data = None
      self.angle = 0
      self.cmd = AckermannDriveStamped()
      self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size = 1)
      self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
    
    def scan(self, data):
      self.data = data
      self.drive()

    def drive(self):
      self.angle = self.find_wall()
      self.cmd.drive.speed = self.VELOCITY
      self.cmd.drive.steering_angle = self.angle
      self.drive_pub.publish(self.cmd)

    def find_wall(self):
      if self.data == None:
        return 0
      left_data = self.data.ranges[33:84]
      right_data= self.data.ranges[17:67]
      min_left = left_data[0]
      for i in left_data:
        if i < min_left:
          min_left = i
      min_right = right_data[0]
      for i in right_data:
        if i < min_right:
          min_right = i
      # neg, pos, neg, pos (45)
      pro_left = 4.0 * (self.DESIRED_DISTANCE - min_left)/self.DESIRED_DISTANCE
      pro_right = 4.0 * (min_right - self.DESIRED_DISTANCE)/self.DESIRED_DISTANCE
      if self.SIDE == 1:  #left wall
        if min_left < self.DESIRED_DISTANCE:
          tempAngle = pro_right
        else:
          tempAngle = pro_right
      else:               #right wall
        if min_right < self.DESIRED_DISTANCE:
          tempAngle = pro_left
        else:
          tempAngle = pro_left
      

      return tempAngle



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
