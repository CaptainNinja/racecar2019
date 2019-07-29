#!/usr/bin/env python

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
  #controls driving
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
      self.data.ranges[100] gives the rightmost lidar point
      self.data.ranges[50] gives the forward lidar point
      """
      #returns the output of your alg, the new angle to drive in

      #Declaring the variables
      middle = self.data.ranges[int(len(self.data.ranges)*0.5)]
      right = self.data.ranges[int(len(self.data.ranges)*0.166)]
      left = self.data.ranges[int(len(self.data.ranges)*0.833)]

      tempAngle = 0
      CONSTANT_DISTANCE = 1
      direction = True; #True is left; False is right
      print(right)
      #Find the closest wall
      #"""

      if right < left:#Inside of the wall

          if middle <= 2.0: #if the gap is closing in
              #Find a wall to turn to first
              #In here

              mid_right = self.data.ranges[int(len(self.data.ranges)*0.33)]
              mid_left = self.data.ranges[int(len(self.data.ranges)*0.666)]

              if mid_left + left > mid_right + right:
                  direction = False
              if direction == False: #Turning in right:
                  tempAngle = 60
              if direction == True:
                  tempAngle = -60
          else:
              if right > CONSTANT_DISTANCE:
                  tempAngle = -30
              if right < CONSTANT_DISTANCE:
                  tempAngle = 30
      else:
          mid_left = self.data.ranges[int(len(self.data.ranges)*0.666)]
          if mid_left >= 5.0:
              #SOMETHING
              print("somethign")    
              tempAngle = 30
          else:
              #SOmething:
              if left > CONSTANT_DISTANCE:
                  tempAngle = 15 
              if left < CONSTANT_DISTANCE:
                  tempAngle = -15


      return tempAngle


if __name__ == "__main__":
  rospy.init_node('wall_follower')
  wall_follower = WallFollower()
  rospy.spin()	
