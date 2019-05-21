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
