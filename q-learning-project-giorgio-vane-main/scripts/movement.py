#!/usr/bin/env python3

import rospy
import moveit_commander
import math
# import the custom message
from q_learning_project.msg import RobotMoveObjectToTag

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3



class RobotMove(object):

    def __init__(self):
        #rospy.init_node('movement')
        
        #rospy.Subscriber("/RobotMoveObjectToTag", RobotMoveObjectToTag, self.complete_pick_up)

        #rospy.Subscriber("/scan", LaserScan, self.approach_obj)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        
        self.move_group_arm.go([0,0,0,0], wait=True)
        self.move_gripper_to_state(1)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        self.stop_distance = 0.2
        self.tag_angle = 0
        self.object_distance = None
        
        
        rospy.sleep(3)

    def move_arm_to_state(self, arm_state):
        """
        Receives an int state specifying the up or down arm_state for the arm to move
        arm_state is 0 for the upwards position, or 1 for the downwards position
        """

        joint1_states = [math.radians(-50.0), math.radians(15.0)]
        joint2_states = [math.radians(0), math.radians(15.0)]

        arm_joint1_goal = joint1_states[arm_state]
        arm_joint2_goal = joint2_states[arm_state]

        self.move_group_arm.go([0, arm_joint1_goal, arm_joint2_goal, math.radians(-30.0)], wait=True)

        self.move_group_arm.stop()

    def move_gripper_to_state(self, grip_state):
        """
        Receives an int state specifying the open or closed state for the
        gripper to move to
        grip_state is 0 for the closed state and 1 for the open state
        """
        if (grip_state == 1):
            gripper_joint_goal = [0.01, 0.01]
        elif (grip_state == 0):
            gripper_joint_goal = [-0.0085, -0.0085]
        
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()


    def approach_obj(self, data):
        close = False
        if (data[573] >= self.stop_distance):
            # Go forward if not close enough to wall.
            self.twist.linear.x = 0.1
        else:
            # Close enough to wall, stop.
            self.twist.linear.x = 0
            print("TIME TO STOP")
            close = True

        self.twist_pub.publish(self.twist)
        return close

    def complete_pick_up(self, msg):
        self.pick_up_obj()
        self.navigate_to_tag(0.5, msg.tag)
        self.put_obj_down()

    def pick_up_obj(self):
        self.move_group_arm.go([0,0,0,0], wait=True)
        self.move_gripper_to_state(1)
        rospy.sleep(5)

        self.move_arm_to_state(1)
        rospy.sleep(5)

        self.move_gripper_to_state(0)
        rospy.sleep(5)

        self.move_arm_to_state(0)
        rospy.sleep(7)

    def put_obj_down(self):
        self.move_arm_to_state(1)
        rospy.sleep(7)
        
        self.move_gripper_to_state(1)
        rospy.sleep(5)

        self.move_group_arm.go([0,0,0,0], wait=True)
        rospy.sleep(5)

    def check_object_distance(self, desired_distance):
        if self.object_distance is not None:
            return self.object_distance <= desired_distance
        return False

    def laser_callback(self, data):
        self.object_distance = data.ranges[573]
        #self.object_distance = min(data.ranges)
        self.tag_distance = data.ranges[0]
        self.tag_angle = 0
    '''
    def navigate_to_tag(self, desired_distance, tag):
        twist = Twist()
        print("navigating to tag") 
        if tag:      
            if self.tag_distance > desired_distance:
                if self.tag_angle > 0.1:
                    twist.angular.z = -0.1
                elif self.tag_angle < -0.1:
                    twist.angular.z = 0.1
                else:
                    twist.linear.x = 0.2
                
                self.twist_pub.publish(twist)
                rospy.sleep(0.1)
            else:
                twist.linear.x = 0
                twist.angular.z = 0
                self.twist_pub.publish(twist)
            
    '''     
        


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    robot = RobotMove()
    rospy.Subscriber("/scan", LaserScan, robot.laser_callback)
    #robot.complete_pick_up()