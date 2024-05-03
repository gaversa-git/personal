#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

class WallFollower:
    def __init__(self):
        """
        Initializes the publisher for /cmd_vel to move the robot and the
        subscriber to receive the messages from the robot's LiDAR
        """
        rospy.init_node('wall_follower')
        self.wall_fol = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.startUp = True

    def callback(self, msg):

        #obj_dsts = list(msg.ranges)
        if (self.startUp):
            self.findWall(msg)
        else:
            self.followWall(msg)



    def followWall(self, msg):
        
        #Initalizes list of sensor values directly in front and 90 degrees to 
        #the right of the bot
        obj_dsts = list(msg.ranges)
        fwd_dsts = obj_dsts[532:542] 
        obj_dsts = obj_dsts[281:291]

        #Initializes variables to store information on objects and their 
        #rotation, 'closest_obj' refers to the object on the right side of bot
        min_closest_obj = 0
        max_closest_obj = 0
        fwd_obj = 0

        min_range = 0
        max_range = 0
        angular_val = 0
        fwd_val = 0
        
        #Records the closest object on the bot's side and its position relative
        #to the bot with min_range and max_range (lower values for min_range 
        #refers to the back end of the bot and higher values closer to the front
        #)
        for i, n in enumerate(obj_dsts):
            if (n != float('inf')):
                if (min_closest_obj == 0 or n < min_closest_obj):
                    min_closest_obj = n
                    min_range = i
                if (max_closest_obj == 0 or n > max_closest_obj):
                    max_closest_obj = n
                    max_range = i

        
        for i, n in enumerate(fwd_dsts):
            if (n != float('inf') and n > fwd_obj):
                fwd_obj = n

        #Wall directly in front of bot -> turn
        if (fwd_obj < 0.45):
            angular_val = 0.2
        #Side wall not close to bot -> continue forward
        elif (min_closest_obj > .35):
            fwd_val = 0.2
        #Lopsided bot relative to side wall -> turn to adjust
        elif (max_closest_obj - min_closest_obj > 0.4):
            #If front end is slanted and further away, adjusts by turning to 
            #wall
            if (max_range - min_range >= 0):
                angular_val = -0.2
            else:
                angular_val = 0.2
        #If side wall is too close to bot and front of bot is the closer side
        # -> turn 
        elif (min_closest_obj < .3 and min_range - max_range > 0):
            angular_val = 0.2
        #No objects in front -> continue forward
        elif (fwd_obj > 0.65):
            fwd_val = 0.2
        #All other conditions -> turn
        else:
            angular_val = 0.2

        #Push values to move bot
        follow_twist = Twist(linear=Vector3(fwd_val, 0, 0),
                            angular=Vector3(0, 0, angular_val))

        self.wall_fol.publish(follow_twist)


        


    def findWall(self, msg):
        """
        Moves the bot straight untill it reaches a wall and then stops it
        """

        #Filters out all sensor values except for the ones directly in front of
        #the bot, which are used to detect a wall in front of the bot
        obj_dsts = list(msg.ranges)
        obj_dsts = obj_dsts[532:542]
        
        closest_obj = 30.0

        for i, n in enumerate(obj_dsts):
            if (n != float('inf') and n < closest_obj):
                closest_obj = n
        

        if(closest_obj > .5):
            straight_twist = Twist(linear=Vector3(1, 0, 0),
                            angular=Vector3(0, 0, 0))
            print(closest_obj)
        else:
            self.startUp = False
            straight_twist = Twist(linear=Vector3(0, 0, 0),
                            angular=Vector3(0, 0, 0))

        self.wall_fol.publish(straight_twist)


    def run(self):
        """
        Runs the robot indefinitely
        """
        rospy.spin()



if __name__ == '__main__':
    node = WallFollower()
    node.run()