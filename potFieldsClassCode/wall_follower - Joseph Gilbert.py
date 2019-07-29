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
    #VELOCITY = #rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = 1 #rospy.get_param("wall_follower/desired_distance")
    behindforce=0
    ktheta=.5
    #maxangle=rospy.get_param("wall_follower/max_steering_angle")
    def __init__(self):
        # Initialize your publishers and
        # subscribers
     #   print(self.maxangle)
        self.data = None    
        self.angle_set = 0
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    def scan(self, data):
        #stores the lidar data so you can work with it
        self.data = data

    #calls function that controls driving
        self.autorun()

    def autorun(self):
        self.fieldpot()
    def drive(self,speed, angle):
        """controls driving"""

    #gets the angle required
        self.angle = angle

    #sets speed and driving angle
        self.cmd.drive.speed = speed
        self.cmd.drive.steering_angle = angle
        #publishes the command
        self.drive_pub.publish(self.cmd)
    def pol2cart(self,r, theta):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return(x, y)
    def cart2pol(self,x, y): #rads from y not x axis
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(x, y)
        return(rho, phi)
        
        
        
    def fieldpot(self):
        points=[]
        for item in self.data.ranges:
            #inverse squer law
            x,y=self.pol2cart((float(item)),(np.pi*3/4-self.data.ranges.index(item))*1.5*np.pi/len(self.data.ranges))
            #type(points)-
	    ddx=x/((item)**3)
            points.append(ddx)
            
        #print("points array is {0}".format(isdr))
        total=np.sum(points)
        #print(total)
        #total[1]+=self.behindforce*(len(self.data.ranges)/3)*1/self.DESIRED_DISTANCE**2
        #tvel, trad =self.cart2pol(total[0],total[1])
        print("total array is {0}".format(total))#. total sum is (r,theta){1},{2}".format(total,tvel,trad))
        self.VELOCITY=1 #tvel
        #angle=self.ktheta*trad
        #print(tvel)
        self.drive(1,total*self.ktheta)
        return 0
        
        
        
    def pid(self):
        #wall on right
        pos=min(self.data.ranges)
        if pos>2*self.DESIRED_DISTANCE:
            self.nowall=True
        if self.nowall:
            if pos<1.2*self.DESIRED_DISTANCE:
                self.nowall=False
            return 0
        p=(self.DESIRED_DISTANCE-pos)
        d=np.cos((len(self.data.ranges)-self.data.ranges.index(max(self.data.ranges)))*1.5*np.pi/len(self.data.ranges))#rads/point
        self.lastsum+=p
        i=self.lastsum
        #print ("kp is {0}. ki is {1}. kd is {2}. the pos is {3} the p is{4} the i is{5} the d is{6}.".format(self.kp,self.ki,self.kd,pos, p,i,d))
        return (p*self.kp+d*self.kd+i*self.ki)
    
    
    
    
    
    def find_wall(self):
    # if lidar data has not been received, do nothing
        if self.data == None:
            return 0
        return self.fieldpot()
    ## TO DO: Find Alg for Wall Following ##
    
        """Lidar data is now stored in self.data, which can be accessed
        using self.data.ranges (in simulation, returns an array).
        Lidar data at an index is the distance to the nearest detected object
        self.data.ranges[0] gives the leftmost lidar point
        self.data.ranges[99] gives the rightmost lidar point
        .self.data.ranges[49] gives the forward lidar point
        """
        #tempAngle=
        
    #returns the output of your alg, the new angle to drive in
        #return tempAngle

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
