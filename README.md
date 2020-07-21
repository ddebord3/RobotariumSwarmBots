# RobotariumSwarmBots
This set of MATLAB code consists of scripts made to run in the Georgia Tech Robotarium (https://www.robotarium.gatech.edu/) environment. It controls swarms of robots moving around their environment as they swarm around moving point targets. The basis of the controls for the robots is Craig Reynolds' paper on "bird-oid" controls found here: http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/. In addition to the intra-flock collision avoidance, flock centering, and velocity matching behaviors listed in the paper, obstacle avoidance with the walls of the environment, other swarms, and extraneous obstacles in the environment was added to make the behavoir of the robots more practical. The robots are controlled at a lower level using a unicylce model approach. 

This code will not run outside the Robotarium or its simulator's environment. The supporting code may be found here: https://github.com/robotarium/robotarium-matlab-simulator. 

# Simulations and Test Runs
The following videos contain test runs of the code being executed either in the simulation or on the actual robots in the robotarium. 

1. Robotarium Test: https://youtu.be/rFuslhNu4nY. This is a test of an earlier version of the code being run on the robots in the robotarium. 

2. Collision Avoidance Test: https://youtu.be/O1DW58kqF9Y. This is the current version of the code being run in a simulation where swarms are likely to cross each other paths. The resulting video demonstrates the robots ability to avoid one another. 

3. Basic Behavoir Test: The following simulation clearly demonstrates the robot's ability to perform basic swarm behavoir (intra-flock collision avoidance, flock centering, and velocity matching) clearly due to the reduced number of obstacles.  

In all videos the following is true of the additional illustrations:

1. There are two swarms identified by the different colored circles being projected around them. The red swarm is following the robot with a black rung around it. The cyan swarm is following the robot with a pale blue ring around it. 

2. The leader robots are being made to follow a set point around the environment. The black ringed robot follows the black * mark while the blue robot follows the blue * mark. 

3. Green objects are miscellaneous objects to be avoided.  

4. The black rectangle marks the boundary of the arena while the black ellipse inscribed within it marks the practical boundary the robots will try to stay within. This control scheme has limitations in cases where the robot is within a sharp corner. To solve this problem, the robots in the simulation will treat the ellipse as the effective boundary, effectively rounding the environments edges. 
