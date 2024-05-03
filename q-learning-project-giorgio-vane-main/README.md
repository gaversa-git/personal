# q_learning_project
# Final Deliverable

** received extension **

## Robot Perception

To complete robot perception we first masked the chosen color of our object with the cv2 library, using a very similar approach to the
lab B line follower script. After finding the target color and turning to it until the target color is approximately centered in the 
camera view, we then use the laser scan message topic of the LiDAR to approach the object. When close enough we again turn until the
sensor reading directly in front of the camera is close enough. Next we can then look for the AR tags and turn until we find one,
retrieving the tag from the aruco dictionary and matching it with the tag we found. This entire process gets carried out through our 
image_callback(), scan_callback(), and find_ar_tag() functions. In order to navigate to the tags, we also implemented a navigate_to_tag_function(). In order to move across states in the q matrix, we call our update_state() and select_action() functions, which choose actions from the q matrix, and check if they are possible given the last action.

## LIDAR 
RPLidar-A1

## Robot Manipulation and Movement

Robot movement in approaching object locations is strongly tied with the robot perception, with turning and approach objects utilizing
the laser scan and an approach to find the closest object. Once an object has been reached (which utilizes the approach_obj() and 
turn_to_obj()), we then have the pick_up_obj() function call for an object directly in front of the bot. This utilizes a combination
of helper functions to do a multi-step pick up, moving the arm and gripper between up and down states and open and closed states respectively. The put_obj_down() is therefore a very similar approach, but in a different order of manipulating the states.

## Q-learning algorithm

In order to select actions for our robot to take, we filtered out invalid actions from the action matrix, in order to ensure only valid actions were taken by the bot. We updated the q-matrix iteratively, using the values in the action matrix to help in only simulating the possible actions at a given state. We determined convergence after 1000 iterations of the q-matrix. We chose the path to receive award by using the action at a given state with the highest reward, and if there were multiple of the same value, we would choose the first of those actions.
## Challenges

One of the major challenges we found was in dealing with the Qlearning matrix where we kept getting incorrect values. To overcome this
we tried to implement our perception code with hardcoded colors until the Q-matrix was complete. The Q matrix required us to rewrite the entire code after receiveing the feedback from the intermediate deliverable. Additionally, the perception was a big challenge, first in
recognizing the colors of the objects and then when dealing with lag. One solution we found after of course tweaking with color values, 
was utilizing a combination of laser scan and image color perception to detect our object, and turning, moving, and turning again as a final adjustment to ensure our object is centered in front of the robot. Another challenge involved the color blue, which we were still unable to detect likely due to the lighting and noise from the environment. Another challenge that could not be fixed in time was the robot still spinning while putting down the object despite the angular velocity being set to 0, which led to me having to pick the objects back up after the robot initially put them down as it would knock down the objects.

## Future Work

To improve our implementation I would optimize the speed that the robot could move at and the flexibility of pick up ranges for the
objects. Mainly changes would be made to movement to make it optimal, but color values and how we approach an object could be adjusted
as well. Building the code to have more safeguards for problems like camera lag and objection recognition would be all of the main
improvements. I would also further look into how to make the integration of the image callback and perception more seamless as we had many booleans in our code which were hard to keep track of.

## Takeaways

    - Image recognition and colors can be very slow and tricky, while the laser scan is more accurate. A combination of both of these
    can be used to circumvent lag in the connection between the turtlebot and the code, which is very common.
    - Arm control works pretty smoothly, and utilizing reusable methods and sufficient delay is the best way to implement it. However,
    ensuring the specific ranges of movement you want and how long it takes to complete an action requires some testing.
    - Keep trying and don't give up. Even after technical issues and issues with our intermediate deliverable, we kept going and were able to submit our project!

## Optimizations

In order to make our code more optimized to circumvent lag, we ran our code without displaying the camera, as it added a lot of lag to our machines.

# Intermediate Deliverable

## Objectives Description

Our objective is to first program a Q-Learning algorithm which will allow us to categorize and quantify states and actions in a robot's environment. Once our Q-matrix has converged, our robot, having perceived the items and target locations around it, will move colored objects to their appropriate AR tags from the learned goal states of the Q-matrix. Our goals involve training a robot to perceive-i.e detect items and drop them off, and to control a robot's ability to pick up items and navigate to their drop off locations.

## High-level Description

We plan to use reinforcement learning in calculating the q matrix. We plan to computationally find the actions a robot should take in order to reach a goal state (AR tags having objects put in front). We trained a q matrix using a Q-learning algorithm while also publishing and subscribing to provided nodes. The actions the robot can take are accessed from self.actions in the starter code. While iterating through the algorithm, invalid state movements are denoted with a -1. Once the matrix has converges, we save it in a csv file for further use in our perception portion, which will require the implementation of code that moves the robot, its arm, and the placement of objects in front of their AR tags.

## Q-Learning Algorithm Description

### Selecting Action States

We iterated through the self.action_matrix in order to populate an array of possible actions for all valid actions and we populated a self.state_actions array that contains the resulting action that was taken from the given state. self.state_actions is populated wirh placeholders that will be replaced if a given action is possible from the current state, and an empty list is added to self.possibilities[i] which will then store all valid actions. After indicating if an action is valid, we add the action to a list of possibel actions for the state at self.possibilities[i], and the resulting state is saved in self.state_actions. At first, we wanted to access actiosn randomly, but found this to be complicated when dealing when moving from one action state to another as these are dependent on one another.

### Updating Q-Matrix

We set a learning rate, a gamma value, and a state value initialized to 0.  If no actions are aavailable at a given state, the state resets to 0. An action denoted as val is randomly chosen from the possible actions at the given state and based on this action, the next action is picked. The values in the q matrix are updated based on the received reward, and the maximum of the next state denoted as following_val. The equation implemented was derived from the course website. The discount rat is adjusted based on the size of the update to the current q value and this metric is used to decide convergence.

### Exit Condition for Algorithm

The loop begins by checking for convergence, which is true if the value of self.discount < .1. The q learning loop is maxxed at 10000 iterations. The loop only breaks when this value of iterations is reached or if the matrix has converged.

### Executing a Path (not currently required)

# Implementation Plan

Vanessa Lois, Giorgio Aversa

To execute the Q-Learning Algorithm, we will use a computational approach as outlined in the project specification. In the algorithm, we initialize our matrix recording all possible world states with potential robot actions. This matrix’s dimensions will correspond to the possible actions and states. After iterating over random selections of each state, and selecting actions randomly from a set of valid options, we will update the q-values using our defined reward metric. We will stop the algorithm when these values converge. When the updates to the q-matrix become trivial (<.01 change across elements), we will consider the matrix as converged. In order to test this, we will try to simulate the changes in q-values in order to determine where values have converged–likely by using a counter which helps us identify convergence across 100 (or more) iterations. For testing we can also print the values of different matrices generated at the end and determine if they are appropriately converged. Additionally, we can sample with a small amount of iterations of the algorithm ato see if it lines up with our expectations.

To implement the robot perception portion of the q-learning project, we can use the functionality of ROS camera-correlated topics (like /scan and /camera) in addition to the OpenCV python library. We will be sure to initialize this environment both after setting up our Q-matrix and with as little noise as possible to ensure that colors are properly identified. In order to have more seamless configuration between the actual turtlebot and the simulator we will be sure to publish specifically to the /camera/rgb/image_raw topic. To identify the colors of the tubes we will use OpenCV to identify them based on color. To identify the AR tags, OpenCV’s aruco library can help us search for the predefined AR tags in the `aruco_dict` dictionary of tags and index the dictionary for the current grayscale image in order to have the robot act accordingly. We will be using this in tandem with the images scanned by the robot’s raspberry pi camera in order to receive the images to compare to the ones indexed in the dictionary. Once the object and tag are identified, their positions will be input into the converged q-learnign algorithm in order to make actions based on the q-matrix. We will test the colored poles to similarly colored objects in order to see how well the colors can be differentiated. We will test the perception portion through simulation as well–which is crucial with respect to the q-matrix. 

In order to pick up and put down the colored objects with the OpenMANIPULATOR arm, we will program the arm in order to pick up the objects based on the results of the perception portion. The robot will navigate to these objects and AR tags using ROS navigation functionality. In order to test the arm, we will program specific arm movements and test the grip strength of the robot to make sure it is accurate. We will also test the robots movement between the tag and the objects by having the robot move to various positions based on AR tags, and we plan to also simulate this process on our machines

Timeline:
Wednesday - Q Learning Algorithm and Convergence
Friday - Q Learning Action Selection
Sunday - Robot Perception
Tuesday - Robot Manipulation
Thursday - Robot Movement

![Q learning gif](https://github.com/Intro-Robotics-UChicago-Spring-2024/q-learning-project-giorgio-vane/assets/86925885/9af46c88-c339-4399-b7b7-d9cdbb3125df)
