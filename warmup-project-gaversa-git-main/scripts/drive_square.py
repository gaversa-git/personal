#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time


class DriveSquare:
    def __init__(self):
        """
        Initializes the publisher and the booleans driveStraight and 
        finishedSquare which tracks when the turtlebot should be driving 
        straight and when the square has been finished.
        """
        rospy.init_node('drive_square')
        self.square_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.driveStraight = True
        self.finishedSquare = False
        

        
    def turnTwist(self):
        """
        Returns a twist used for turning the turtlebot.
        """
        my_lin = Vector3(0.0,0.0,0.0)
        my_ang = Vector3(0.0,0.0,0.20)
        turn_twist = Twist(linear=my_lin, angular=my_ang)

        return turn_twist

    def straightTwist(self):
        """
        Returns a twist used for moving the turtlebot in a straight line.
        """
        my_lin = Vector3(0.15,0.0,0.0)
        my_ang = Vector3(0.0,0.0,0.0)
        straight_twist = Twist(linear=my_lin, angular=my_ang)

        return straight_twist


    def run(self):
        """
        Function for running the turtlebot to complete a square path and then
        stop.
        """
        
        #Obtains the turning and straight twists
        turn_twist = self.turnTwist()
        straight_twist = self.straightTwist()

        #Pauses to allow publisher enough time to setup
        rospy.sleep(5)

        #Records time to 2 variables
        initial_start = time.time()
        start = time.time()
        r = rospy.Rate(1)
        
        #Publishes each directional twist for a certain amount of time before
        #changing it
        while not self.finishedSquare:
            
            if(self.driveStraight):
                self.square_pub.publish(straight_twist)
                rospy.sleep(1)
            else:
                self.square_pub.publish(turn_twist)
                rospy.sleep(1)

            cur = time.time()

            if ((cur - start) > 6 and self.driveStraight):
                start = time.time()
                self.driveStraight = False
            elif ((cur - start) > 8 and not self.driveStraight):
                start = time.time()
                self.driveStraight = True

            #Exits the loop after 56 seconds have elapsed
            if ((cur - initial_start) > 56):
                self.finishedSquare = True
        
        #Stops the turtlebot
        self.square_pub.publish(Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
        ))
            

if __name__ == '__main__':
    node = DriveSquare()
    node.run()

