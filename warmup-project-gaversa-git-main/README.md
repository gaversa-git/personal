# warmup_project

Drive In a Square

    High-Level: 
    The task was to have the turtlebot drive in a square path. Challenges were
    tuning the movement instructions with the environment in order to keep the
    turtlebot on track and not overturn or go too far. My approach was to
    stitch together valid striaght drive instructions with turn instructions
    and then repeat them in alternating order--with valid meaning the 
    turtlebot would not go too far or overturn.
    
    
    Code Explanation: 
    My code was structured in a class containing a constructor initializing the
    publisher, two functions that created the twist instructions for moving in a
    straight line and turning, and then a run function that would send all the
    instructions and perform them in the correct order to complete a square
    circuit. The turnTwist() and straightTwist() are where a user could modify
    the speed that each instruction is performed at. The run() function handles
    the timing and publishing of instructions that have been measured with the
    existing speeds of each twist in order to complete a square circuit.
        Note: All LiDAR models will work with this code, since it does not rely
        on the scan or camera of the robot to execute.

    # My GIF (Link to google drive)
    https://drive.google.com/file/d/1BeBS_LXxURfHorGKTvaNoo-vabmTc0wz/view?usp=sharing

Person Follower

    High-Level:
    The task was to have the turtlebot follow a person which can be simplified as 
    approaching the closest obejct that is picked up in the 360 degree radius around
    the turtlebot. To do this, we would process all the object reading from the 
    LiDAR and pick the closest one and then interpret its position relative to
    the turtlebot in order to publish controls for the bot.

    Code Explanation:
    In the callback function which would loop and receive the subscribed scan
    data from the turtlebot, this data would then be processed to select the
    closest object and its relative rotation (by noting the index position in
    the list of object distances) and passing those two values in to 
    processControl() which would then issue the controls by dynamically
    adjusting them based on how far the robot needed to move such that it
    would be faster for further objects and slower for closer objects to
    account for error in the data readings.
    
    
    GIFs
    ![Part 1/5](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/bac77740-81db-47f0-9f50-39095e3cb590)
    ![Part 2/5](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/f922cb13-07a6-4260-b1d3-9820f0948e48)
    ![Part 3/5](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/fd27f0f5-999e-4ba1-ab41-0c2d7246a006)
    ![Part 4/5](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/fdea5fdf-a45e-4783-9e95-47c5cc709684)
    ![Part 5/5](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/892d2b79-ee94-4e7d-bc92-5d5dfe48b682)

Wall Follower

    High-Level: 
    The task was to have a turtlebot follow along the walls of a room at a fixed distance, 
    accounting for corners and curves in the wall. My approach was to sense whether there 
    was wall in front of the bot and combine forward movements with rotations to adjust
    the bot's position.

    Code Explanation:
    I split the code into two main functions that would get called back and pick up on
    the LiDAR sensor readings. The findWall() function would first run on start up to
    simply move the robot and stop it once it found a wall. Then, once there it would
    run followWall() which would follow along the wall by calculating with its forward 
    and side sensors whether it needed to rotate or could safely progress while keeping 
    sufficient distance from the wall. Additionally, the callback function was the
    function invoked by the scan subscriber and would loop and pass in the message
    data of object ranges to each function as well as disable the findWall() and 
    switch to the followWall() code when appropriate.
        Note: Code was built around the 'RPLIDAR-A1' LiDAR model
    
    The following GIFs show the robot following along a curved series of walls:

    ![CurveWall Part 1/4](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/eb6256f1-86bc-4ef7-90fb-4b14a016897a)
    ![CurveWall Part 2/4](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/2e6872fb-bdca-4cab-85d3-605c7e4b155e)
    ![CurveWall Part 3/4](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/e0ede365-11ff-421f-8033-67ad26b66cb4)
    ![CurveWall Part 4/4](https://github.com/Intro-Robotics-UChicago-Spring-2024/warmup-project-gaversa-git/assets/114627557/ffd4416b-b5c5-460b-83fc-85195e2e16a8)

Challenges

    Challenges of this project can be broken down into phases, of learning the way to
    use /cmd_vel and then to use /scan, at least in the case of the DriveSquare and 
    PersonFollower portions. /cmd_vel was easier to pick up and more a matter of timing
    the controls appropriately. /scan involved more trouble in parsing the LaserScan()
    class and understanding the data we wanted from it. After this point, I struggled
    with getting my /scan data to update quickly enough and worked through possible ways
    of slowing down my code to allow time to update before finally realizing the 
    solution which was copying the data into a list variable and then accessing the
    list variable in my for loop rather than accessing the class's value directly
    with each iteration. Finally, for the FollowWall phase, the trouble was again
    in filtering the data properly with the controls, and finding a way to measure
    the robot's position as parallel to the wall as well as when it needed to turn
    versus when it could proceed forward.

Future Work

    I feel confident with both the PersonFollower and Drive in a Square portions
    so with more time I would focus on the Follow Wall function. The values I
    used are somewhat inaccurate and with the far larger number of edge cases I'm
    sure the code would run into small problems with approaching walls to closely 
    at times. So with more time I would split the code into a isParallel function
    which ensures the robot is parallel to the wall and proceeds forwards. With an
    additional function I think the logic of my if statements would also be easier 
    to understand and finally I could also implement the rotation to occur at the
    same time as the forward movement although this might introduce more error and
    I'm not sure if it is necessary or beneficial.

Takeaways

    - Accessing the data and copying it is important to improve the speed of parsing.
    Noise/inaccuracy is already a big part and so finding ways to minimize the latency
    or time to access large data is needed in order to get more accurate and updating
    information.

    - Spliting code into more functions is always beneficial and helps direct the logic
    of the code better. With specific functions that you can rely on to leave the
    turtlebot in a specific state it is easier to debug as well as improve upon the
    code in the future.











