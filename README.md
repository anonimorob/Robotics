# Robotics

INVERSE KINEMATICS
Inverse kinematic (IK) algorithm for 3 DOF SCARA robot.
To visualize the robot, run: 

roslaunch kinematics_assignment scara_launch.launch

An rviz window will open:
![image](https://github.com/anonimorob/Robotics/assets/115708199/dbd0c2f5-7801-48a8-a6ce-14e64d61b800)
![image](https://github.com/anonimorob/Robotics/assets/115708199/5d10581b-00a3-435a-9bcf-988ab447516c)



PLANNING
Dubin's Car
After ensuring that you have a working version of Python 3, you can obtain the source code by  downloading the repository from:

Files/assignemnt3_planning2023.zip

Implementation of a robotic planning method in order to drive a Dubins car, with the dynamics:

x[t+1]     = x[t]     + cos(theta[t])
y[t+1]     = y[t]     + sin(theta[t])
theta[t+1] = theta[t] + tan(phi[t])

from an initial position (x0,y0) to a target position (xt, yt), while avoiding both collisions with obstacles and venturing out of bounds.

The state variables are:

x: horizontal position
y: vertical position
theta: heading angle (direction of travel)
And, the sole control variable is the steering angle phi ∈ [-pi/4, pi/4] (with respect to the direction of travel).

Note: we refer to the state as (x, y, theta) and the position as (x, y).
Note: each steering angle controls[i] is considered to be constant between times[i] and times[i+1], so controls must be one element shorter than times, i.e. len(controls) == len(times) - 1; the initial time must be zero, i.e. times[0] == 0; 
the time list must be spaced by ≥0.01 seconds;
each steering angle must be admissible, i.e. -pi/4 <= controls[i] <= pi/4;
the time sequence must increase, i.e. times[i+1] > times[i];
the intial heading angle in evaluation is zero, i.e. theta=0;
the obstacles, intial positions, and target positions are randomised, so hard-coded solutions will not work;
the Car object car can not be altered in solution(car).


MAPPING
Mapping is one of the core competencies of truly autonomous robots. Autonomous robots can use maps in a number of different ways, search and rescue robots can use them to make sure they search through the whole building instead of moving in between the same rooms over and over, autonomous cars use them in order to find a path that leads to the desired location, multicopters can use the maps to localize themself in order to stay in the air.

In many situations we cannot assume that the robot can be given a map in advance. Even if there are maps available, such as blueprints for a building, they are not always useful for the robot and might be incorrect (too old, building collapsed, etc). Therefore it is of great benefit if the robot can construct a map by itself from scratch, which is exactly what we will do in this assignment.
Occupancy grid mapping is one of many mapping algorithms. Here the world is represented as a grid, where each cell of the grid corresponds to an area in the world. The value of the cell can tell us if the area is free, occupied, unknown, or something else (which you will see in the C-part of this assignment).

The occupancy grid is characterized by the number of cells and the resolution. More cells means that it is possible to map a larger area. In this assignment we will work with a 2D occupancy grid but 3D grid maps are often used as well. The resolution describes how big of an area each cell covers. If the resolution is 5cm then one cell in a 2D grid covers a 25cm² area. 


Argument             	Purpose
angle_min	            Start angle of the scan in radians
angle_max	            End angle of the scan in radians
angle_increment     	Angular distance between measurements in radians
range_min	            Minimum range value in meter*
range_max	            Maximum range value in meter*
ranges*	              Range data in meter (values <= range_min or >= range_max should be discarded)

The bearing for the first range in ranges (ranges[0]) is angle_min. The bearing for the second range in ranges (ranges[1]) is angle_min + angle_increment and so on. This way you can get both the bearing and the range.

* It should really be (values < range_min or > range_max should be discarded), but due to a small mistake in making the assignment it is now (values <= range_min or >= range_max should be discarded).


  Download the ROS package from here Download herein your ROS workspace and run catkin_make. Remember to source the devel/setup.bash if needed.

  To run the code:
  You have two options for how to run your code in this assignment, the first (using ROS) is highly recommended since you get to see in real time what your code is doing.

Using ROS (from rosbag)
This approach is how you normally work with ROS. We have saved some rosbags, located in mapping_assignment_metapackage/mapping_assignment/bags/, that contain sensor data from a robot which has been moving in an unexplored area. Using this approach you will be able to see the robot in real time mapping the environment, which can help you when you are debugging.

For this you have to open four terminals:

Terminal 1: roscore
A roscore has to be running for ROS nodes to be able to communicate with each other.

It is good practice to start the roscore in a separate terminal, even though a roscore is automatically started when you use roslaunch (note that a roscore is not automatically started when you use rosrun). This is because you might have to restart the launch file that you started with roslaunch, this will cause the roscore to terminate and you have to restart everything.

Terminal 2: roslaunch mapping_assignment play.launch
This launches RVIZ for you so that you can see how the robot moves and how the map is being updated by your code.

Terminal 3: rosbag play --clock BAGFILE
Terminal 4: rosrun mapping_assignment main.py
This runs your code.

NOTE: If you restart the rosbag you should also restart this, since the map is never deleted otherwise between different runs.

