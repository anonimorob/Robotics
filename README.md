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



