#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random
from q_learning_project.msg import QLearningReward, QMatrix, RobotMoveObjectToTag
import cv2
import cv_bridge
from sensor_msgs.msg import Image, LaserScan
import moveit_commander
from collections import deque
from q_learning import QLearning
from geometry_msgs.msg import Twist
from movement import RobotMove
import pandas as pd

#home_path = "/home/gaversa/catkin_ws/src/q_learning_project/"
home_path = "/home/vlois/catkin_ws/src/intro_robo/q_learning_project/"
path_prefix = os.path.dirname(__file__) + "/action_states/"

class RobotPerception():
    def __init__(self):
        rospy.init_node('perception')
        self.bridge = cv_bridge.CvBridge()
        self.init_variables()
        self.load_q_matrix_and_actions()
        self.action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        self.prop_ctrl = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.spin()

    def init_variables(self):
        # initialize all needed variables, i put them here to avoid clutter with the actual init
        self.colors = ['pink', 'green', 'blue']
        self.current_state = 0
        self.current_action = None
        self.isCentered = False
        self.isClose = False
        self.hasObject = False
        self.object = None
        self.tag = None
        self.movement = RobotMove()
        self.completed_actions = []

    def load_q_matrix_and_actions(self):
        # use os and load txt to get action data and our q matrix
        q_path = os.path.join(home_path, 'finished_matrix.csv')
        action_path = os.path.join(path_prefix, "actions.txt")
        self.q_matrix = np.loadtxt(q_path, delimiter=' ', dtype=float)
        action_data = np.loadtxt(action_path, dtype=int)
        
        self.actions = [{
            "object": self.colors[action[0]], 
            "tag": action[1]  
        } for action in action_data]
        
        self.select_action()


    def select_action(self):
        # get the biggest action at a given state in q matrix
        action_index = np.argmax(self.q_matrix[self.current_state])
        self.current_action = self.actions[action_index]
        
        twist = Twist()

    def update_state(self):
        # after action is  completed, update the state
        
        if not self.hasObject:
            # if the object has been put down successfully, add it to the list of completed actions 
            rospy.loginfo(f"Completed placing {self.current_action['object']} at tag {self.current_action['tag']}.")
            self.completed_actions.append((self.current_action['object'], self.current_action['tag']))

            found_new_action = False
            next_indices = np.argsort(-self.q_matrix[self.current_state]) 
            for index in next_indices:
                # get the next action
                next_action = self.actions[index]
                # check if it is in the list already, if not update the value
                if (next_action['object'], next_action['tag']) not in self.completed_actions:
                    self.current_state = index
                    self.current_action = next_action
                    found_new_action = True
                    break

            if not found_new_action:
                rospy.loginfo("All possible actions have been completed or no valid new action found.")
        else:
            rospy.loginfo("Current task not yet completed, continuing current action.")

    def get_mask(self, hsv, color):
        # set up the color boundaries
        bounds = {
            'pink': ([130, 50, 50], [180, 255, 255]),
            'green': ([30, 50, 150], [40, 255, 155]),
            'blue': ([85, 50, 150], [95, 255, 155])
        }
        lower, upper = bounds[color]
        lower = np.array(lower, dtype="int")
        upper = np.array(upper, dtype="int")
        # generate the mask based on the lower and upper
        return cv2.inRange(hsv, lower, upper)

    def image_callback(self, msg):
        # get image, hsv, object color and mask
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        current_object_color = self.current_action['object']
        mask = self.get_mask(hsv, current_object_color)
        if not self.hasObject:
            # if the object isnt in grasp, use the image composition to seacrh for object
            h, w, d = image.shape
            search_top = int(3*h/4)
            search_bot = int(3*h/4 + 20)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                error = cx - w / 2
                # if the image is not centered within a certain threshold, turn
                if abs(error) > 10:
                    twist = Twist()
                    twist.angular.z = -0.1 if error > 0 else 0.1
                    self.prop_ctrl.publish(twist)
                elif abs(error) <= 10:
                    # if object is centered set the booleans to use in scan callback
                    rospy.loginfo("Object centered!")
                    self.isCentered = True
                    self.hasObject = False
            else:
                # else turn
                rospy.loginfo("Searching for object...")
                twist = Twist()
                twist.angular.z = 0.1 
                self.prop_ctrl.publish(twist)

            self.latest_image = image
            self.new_image_flag = True

        if self.hasObject:
            # if the object is in grasp, use similar logic as above to center the ar tag within \
            # a certain threshold
            x_center = self.find_ar_tag(image)
            h, w, d = image.shape
            if x_center != -1:
                error = x_center - w / 2
                if abs(error) < 10:
                    rospy.loginfo("AR Tag centered, moving forward...")
                    twist = Twist()
                    #if the error is less than 10, ar tag is centered and should be navigated to
                    twist.linear.x = 0.1
                    self.prop_ctrl.publish(twist) 
                    self.navigate_to_ar_tag(image, self.find_ar_tag(image))
                    # set booleans
                    self.hasObject = True
                    self.isCentered = True  
                else:
                    # else, turn
                    twist = Twist()
                    twist.angular.z = -0.1 if error > 0 else 0.1
                    self.prop_ctrl.publish(twist)
            else:
                # same here, may need to remove for redundancy
                rospy.loginfo("AR Tag not found, searching...")
                twist = Twist()
                twist.angular.z = 0.1
                self.prop_ctrl.publish(twist)

            self.latest_image = image
            self.new_image_flag = True
    
    def scan_callback(self, data):
        if self.isCentered:
            if not self.hasObject:
                # use lidar to move bot
                front_distance = data.ranges[len(data.ranges) // 2]  
                if front_distance < 0.25:
                    # if bot too close, stop bot
                    rospy.loginfo("Object within reac.")
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.prop_ctrl.publish(twist)
                    # pick object up
                    self.movement.pick_up_obj()
                    self.hasObject = True
                    self.isCentered = False
                elif front_distance >= 0.25:
                    # if bot too far, approach slowly
                    rospy.loginfo("Object too far, approaching...")
                    twist = Twist()
                    twist.linear.x = .1
                    self.prop_ctrl.publish(twist)
            elif self.hasObject:
                # if bot has object 
                front_distance = data.ranges[len(data.ranges) // 2]  
                if front_distance < 0.8:
                    # if bot close to the tag, stop and put down the object
                    rospy.loginfo("Tag within reach")
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.prop_ctrl.publish(twist)
                    # put object down 
                    self.movement.put_obj_down()
                    # put arm up
                    self.movement.move_arm_to_state(0)
                    twist = Twist()
                    twist.angular.z = 0
                    self.prop_ctrl.publish(twist)
                    # reset booleans
                    self.hasObject = False
                    self.isCentered = False
                    # update to the next possible state
                    self.update_state()
                elif front_distance > 0.8:
                    # else, approach tag
                    rospy.loginfo("Tag ahead, moving forward slowly...")
                    twist = Twist()
                    twist.linear.x = .01
                    twist.angular.z = 0
                    self.prop_ctrl.publish(twist)

    
    def find_ar_tag(self, image):
        # use the corners from the 4x4 aruco array to help retrieve the center image point
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(grayscale, aruco_dict, parameters=params)
        if ids is not None:
            for idx, tag_id in enumerate(ids):
                # if the tag id matches the current tag, set the center to the average x value
                # else return -1
                if tag_id[0] == self.current_action['tag']:
                    corner_info = corners[idx][0]
                    x_center = np.mean(corner_info[:, 0])
                    return x_center
        return -1

    def navigate_to_ar_tag(self, image, ar_tag_center):

        w = image.shape[1]
        error = ar_tag_center - w // 2
        twist = Twist()
        # if centering error is too large, turn at a proportional rate 
        if abs(error) > 40:
            twist.angular.z = -0.001 * error
        else:
            # move forward slowly
            twist.linear.x = 0.1
            twist.angular.z = 0
            print("moving forward")
        self.prop_ctrl.publish(twist)
    

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rp = RobotPerception()
    rp.run()
