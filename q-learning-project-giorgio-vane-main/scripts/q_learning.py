#!/usr/bin/env python3
"""
import cv2
# load DICT_4X4_50
"""
import rospy
import numpy as np
import os
import random
from q_learning_project.msg import QLearningReward, QMatrix, RobotMoveObjectToTag
import cv2
import rospy, cv_bridge
from sensor_msgs.msg import Image, LaserScan
import moveit_commander
from collections import deque


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
      
        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))
    

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        # initialize matrices
        self.q = np.zeros((64, 9), dtype=int)
        # set up subscriber and publishers
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_callback)
        self.matrix_publisher = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.action_publisher = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        self.current_state = 0
        self.q_changes = deque(maxlen=100)
        self.convergence_threshold = 0.01
        self.current_action_index = None
        self.current_next_state = None
        self.state_visits = np.zeros(len(self.states), dtype=int)
        self.converged = False
        self.reward = 0

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a csv file once it is done to
        # avoid retraining
        
        #need to find a way to make this general 
        #path = "/home/gaversa/catkin_ws/src/q_learning_project/finished_matrix.csv" 
        path = "/home/vlois/catkin_ws/src/intro_robo/q_learning_project/finished_matrix.csv" 
        np.savetxt(path, self.q)
        print("matrix saved")


    def get_possible_actions(self, state):
        # get all possible actions based on actions matrix
        state_actions = []
        
        for action_index, next_state in enumerate(self.action_matrix[state]):
            next_state = int(next_state)
            if (next_state == -1):
                continue
            else:
                state_actions.append((action_index, next_state))
        #Returns list of tuples containing (state, action)
        return state_actions
        
            

    def print_q_matrix(self):
        # print q matrix to debug
        print(self.q)

    def reward_callback(self, data):
        # get reward metrics, one from data and one default
        self.reward = data.reward

    def q_matrix(self):
        # learnign rate, gamma, and start state
        a = 1
        g = .8
        j = 0
        start_state = 0
        
        num_iterations = 1000
# set convergence for 1000 iterations
        for _ in range(num_iterations):
            state = start_state
            while True:
                possible_actions = self.get_possible_actions(state)
                if not possible_actions:
                    break

                next_state, action_index = random.choice(possible_actions)
                # perform action
                move_bot = RobotMoveObjectToTag()
                move_bot.robot_object = self.actions[action_index]["object"]
                move_bot.tag_id = self.actions[action_index]["tag"]
                self.action_publisher.publish(move_bot)

                rospy.sleep(0.1)
                # update q
                max_future_q = np.max(self.q[next_state])
                self.q[state, action_index] += a * (self.reward + g * max_future_q - self.q[state, action_index])
                state = next_state

if __name__ == "__main__":
    node = QLearning()
    node.q_matrix()
    node.save_q_matrix()
    node.print_q_matrix()

