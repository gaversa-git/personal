#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

class PersonFollower:
    def __init__(self):
        """
        Initializes the publisher for /cmd_vel to move the robot and the
        subscriber to receive the messages from the robot's LiDAR
        """
        rospy.init_node('person_follower')
        self.person_fol = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)


    def callback(self, msg):
        """
        Function for processing the messages received from /scan, assigning
        the location of the closest object scanned, and passing the information
        to processControl()
        """
        
        #Initializing variables to record the distance and relative rotation of
        #the closest object
        closest_obj = 30.0
        rel_rot = 0

        #Processes the msg.ranges into a list for quicker access in the for loop
        data = list(msg.ranges)

        #Finds the closest object from the list of objects scanned
        for i, n in enumerate(data):
            if (n != float('inf') and n < closest_obj):
                closest_obj = n
                rel_rot = i


        self.processControl(rel_rot, closest_obj)


    def processControl(self, rot, dist):
        """
        Dynamically moves robot based on how far its target distance and 
        rotation is from the robot
        """
        fwd_modifier = 0
        rot_modifier = 0
        
        #rot follows the LiDAR number of sensor readings, so it ranges from 
        #0-1147 and scales proportional to 360 degrees, when the rot is further
        #from 570-590 (the sensor range directly in front of the turtlebot)
        #the angular speed will increase to reach that point quicker
        if (rot > 570 and rot < 590):
            rot_modifier = 0
        elif (rot <= 200):
            rot_modifier = -1
        elif (rot <= 540):
            rot_modifier = -0.7
        elif (rot <= 570):
            rot_modifier = -0.15
        elif (rot >= 947):
            rot_modifier = 1
        elif (rot >= 720):
            rot_modifier = 0.7
        elif (rot >= 606):
            rot_modifier = 0.4
        else:
            rot_modifier = 0.15

        #The fwd_modifier works the same as the rot_modifier just for distance
        #and linear velocity
        if (dist > 5):
            fwd_modifier = 1.5
        elif (dist > 3):
            fwd_modifier = 0.7
        elif (dist > 1.4):
            fwd_modifier = 0.32
        elif (dist > 1):
            fwd_modifier = 0.25
        elif (dist > .7):
            fwd_modifier = 0.14
        elif (dist > .4):
            fwd_modifier = 0.09
        else:
            fwd_modifier = 0
        
        ctrl_twist = Twist(linear=Vector3(fwd_modifier, 0, 0),
                            angular=Vector3(0, 0, rot_modifier))

        self.person_fol.publish(ctrl_twist)


    def run(self):
        """
        Runs the robot indefinitely
        """
        rospy.spin()



if __name__ == '__main__':
    node = PersonFollower()
    node.run()