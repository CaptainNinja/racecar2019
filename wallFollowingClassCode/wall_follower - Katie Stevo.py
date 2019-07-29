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

      # Initialize your publishers and subscribers

    print("Checkpoint 1")

    self.data = None    

    self.cmd = AckermannDriveStamped()

    self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)

    self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

  def scan(self, data):

    print("Checkpoint 2")

  # stores the lidar data so you can work with it

    self.data = data

  # calls function that controls driving

    self.drive();

  def drive(self):

    print("Checkpoint 3")

  # controls driving

  # gets the angle required

    self.angle = self.find_wall()

    self.VELOCITY = 0.5

  # sets speed and driving angle

    self.cmd.drive.speed = self.VELOCITY

    self.cmd.drive.steering_angle = self.angle

      #publishes the command

    self.drive_pub.publish(self.cmd)

  def find_wall(self):

  # if lidar data has not been received, do nothing

    print("Checkpoint 4")

    if self.data == None:

      return 0

  # decides whether to go left or right

    index=0

    value=self.data.ranges[0]

    for idx, val in enumerate(self.data.ranges):

      if val < value:

        index = idx

        value = val

    print("new index: ", index, " new value: ", value)



    distanceFromWall = self.DESIRED_DISTANCE

    index = 0
    smallest = 0

    for i in range(40):
    	if i == 0:
    		smallest = self.data.ranges[i]
    		index = i 
    	else:
    		if self.data.ranges[i] < smallest:
    			smallest = self.data.ranges[i]
    			index = i 



    if smallest > 2:
    	print("right")
    	return 1

    else:
    	print("left")
    	return -1
 


    '''

    if self.data.ranges[80] < self.data.ranges[85]:

      return 1

    if self.data.ranges[80] > self.data.ranges[85]:

      return -1

    return 0

    '''

    '''

    indexlen = 0

    use = 0

    if self.data.ranges[48] > 2 and self.data.ranges[52] > 2:

      return 0

    if index < 50:

      use = 39

      indexlen = self.data.ranges[use]

      if (indexlen < 1.15473 * distanceFromWall):

        return 1

      if (indexlen > 1.15473 * distanceFromWall):

        return -1

    if index > 50:

      use = 61

      indexlen = self.data.ranges[use]

      if (indexlen < 1.15473 * distanceFromWall):

        return -1

      if (indexlen > 1.15473 * distanceFromWall):

        return 1
    return 0

    '''
    '''
    #hugs a wall with this code

    indexlen = 0

    use = 0

    if index < 50:

      use = 33

      indexlen = self.data.ranges[use]

      if (indexlen < 1.4142 * distanceFromWall):

        return 1

      if (indexlen > 1.4142 * distanceFromWall):

        return -1

    if index > 50:

      use = 67

      indexlen = self.data.ranges[use]

      if (indexlen < 1.4142 * distanceFromWall):

        return -1

      if (indexlen > 1.4142 * distanceFromWall):

        return 1


    return 0

    '''

    '''

    if index < 12:

      #print("one")

      return -1

    if index > 13 and index < 50:

      #print("two")

      return 1

    if index > 50 and index < 89:

      #print("three")

      return -1

    if index > 89:

      #print("four")

      return 1

    return 0

    '''

  # Lidar data is now stored in self.data, which can be accessed

  # using self.data.ranges (in simulation, returns an array).

  # Lidar data at an index is the distance to the nearest detected object

  # self.data.ranges[0] gives the leftmost lidar point

  # self.data.ranges[100] gives the rightmost lidar point

  # self.data.ranges[50] gives the forward lidar point

  

  # returns the output of your alg, the new angle to drive in

  # return tempAngle
if __name__ == "__main__":

  rospy.init_node('wall_follower')

  wall_follower = WallFollower()

  rospy.spin()

