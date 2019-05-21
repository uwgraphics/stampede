# stampede


Welcome to Stampede! This solver implements the methods discussed in our paper <i> STAMPEDE: A Discrete-Optimization Method for Solving Pathwise-Inverse Kinematics </i> (https://graphics.cs.wisc.edu/Papers/2019/RMG19/19_ICRA_Stampede.pdf)


Stampede is a discrete-optimization technique for finding feasible robot arm trajectories that pass through provided 6-DOF Cartesian-space end-effector paths with high accuracy, a problem called pathwise-inverse kinematics. For example, our solver could find a smooth, feasible robot trajectory (i.e., the avoids self-collisions, kinematic singularities, and joint-space discontinuities throughout the motion) that passes through an end-effector Cartesian path with neglible error.  The output
from our method consists of a path function of joint-angles that best follows the provided end-effector path function, given some definition of "best", such as a minimum-velocity trajectory.    

To start using the solver, please follow the step-by-step instructions down below.

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (email: rakita@cs.wisc.edu, website: http://pages.cs.wisc.edu/~rakita)
We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive or negative experiences in using it.

<b> Citation </b>

If you use our solver, please cite our ICRA paper STAMPEDE: A Discrete-Optimization Method for Solving Pathwise-Inverse Kinematics

<pre>
@inproceedings{rakita2019,
  title={Stampede: A Discrete-Optimization Method for Solving Pathwise-Inverse Kinematics},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2019},
  organization={IEEE}
}
</pre>


<b> Dependencies </b>

Before going through the tutorial steps below, ensure that all of these dependencies are installed!

In order to use the Stampede solver, you will first need to install our inverse kinematics solver called RelaxedIK, as well as install all of its dependencies.  The RelaxedIK solver can be found here: https://github.com/uwgraphics/relaxed_ik .  The dependencies for RelaxedIK can be found in the dependencies section in the README.

NOTE: At the present time of writing this setup guide, the current compatible version of RelaxedIK is on the dev branch NOT the master branch.  To install the dev branch version of RelaxedIK, use the following command:

<pre> git clone -b dev https://github.com/uwgraphics/relaxed_ik.git </pre>


<b> Tutorial </b>

Before following the steps in this tutorial, please ensure that all of these dependencies listed above are installed!

Using the stampede solver involves four steps:

1. First, run the following command:
<pre> rosrun stampede create_relaxedik_path.py  </pre>

This will create a file in the Stampede/Config folder called relaxedik_path that will contain the path to the relaxed_ik package src folder.  Note that this step only has to be done once.  Once this file is in place on your system, it does not have to be created again (unless the relaxed_ik package is moved).  


2. Next, we have to specify the end-effector path that the robot should try to follow.  To do this, place a file in the Stampede/InputMotions folder with the following format:

      time;ee_goal_position.x,ee_goal_position.y,ee_goal_position.z;ee_goal_quaternion.w,ee_goal_quaternion.x,ee_goal_quaternion.y,ee_goal_quaternion.z

  Here, time corresponds to the time point (in seconds) that the robot should pass through the corresponding end effector       pose goal, and ee_goal_position.[] and ee_goal_quaternion.[] correspond to the position and orientation goal components       that the end-effector should try to achieve at that time.  NOTE: the position and orientation goals are represented           RELATIVE to the initial state specified in the robot info file in the relaxed_ik package.  For example, a position goal of   [0,0,0] and orientation goal of [1,0,0,0] would correspond to the robot's end-effector exhibiting the same pose as the       robot's end-effector in its initial state.

3. Next, set the relevant parameters in the stampede.launch folder (in the launch folder at the top level of the package), and start the solver using the following command:

<pre> roslaunch stampede stampede.launch  </pre>

The parameters in this file are:

*robot_info_file  (this corresponds to the info_file_name for a particular robot platform set up in the relaxed_ik pacakage)

*input_motion_file  (this correspond to the name of the task file specified in step 2 above)

*scaling_factor (this value uniformly scales up the trajectory points specfieid in step 2 above.  Note that this scaling only applies to the position goals and not the orientation goals)

The solver takes about 30 seconds to three minutes to find the best solution.  This time depends on how many paths the solver deems are still worth keeping active throughout the global optimization process.  Also, keep in mind that the solver is written in the Julia programming language, which currently takes a little while (~20 seconds) for the solver to start. Julia code is VERY fast once it starts up, but it takes a bit of overhead time to do its JIT compilation.  Unfortunately, there's little we can do at this point to eliminate this JIT compilation time the first time the solver starts up (though, we are hoping compilation times get better in general in future versions of the Julia programming language!).  We'll also keep looking into ways to get around this issue in the meantime.

Note that Stampede may determine that no solution exists to the current pathwise-IK problem.  If this is the case, the solver will print that no solution is found, and output the best partial trajectory.

4. Finally, the solver outputs its solution in the Stampede/OutputMotions folder in the file last_trajectory.stampede.  This file will be overwritten on every solve, so make sure to rename this file if you'd like to save a trajectory for later use!  To play back the trajectory in rviz, use the following command:

<pre> roslaunch stampede play_trajectory.launch  </pre>

By default, this will play the file last_trajectory.stampede.  However, if you'd like to play another trajectory file in the OutputMotions folder, just set the trajectory_name argument in the launch file to the desired trajectory name.


<b> Coming Soon </b>
This code was just released on May 21, 2019 as part of the presentation of the corresponding paper at the ICRA conference.  If you experience any bugs, please let me know!  I will likely be updating the code daily during this code roll-out, so feel free to check back as any bugs are ironed out or new features are added in.  

If you have feedback based on your experience using the solver (positive or negative), please email me at rakita@cs.wisc.edu.  Thanks!






