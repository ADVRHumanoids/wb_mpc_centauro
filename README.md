# wb_mpc_centauro
This repo implements the whole-body MPC presented in the following publication: <br />
*I. Dadiotis, A. Laurenzi, N. Tsagarakis*
**"Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped" in 2023 IEEE-RAS International Conference on Humanoid Robots (Humanoids)**
```
@misc{dadiotis2023wholebody,
      title={Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped}, 
      author={Ioannis Dadiotis and Arturo Laurenzi and Nikos Tsagarakis},
      year={2023},
      eprint={2310.02907},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
paper: https://ieeexplore.ieee.org/document/10375215 (or open source https://arxiv.org/abs/2310.02907) <br />
Video: https://youtu.be/8XIAtw4201o     <br />

## Introduction
This repo implements Model Predictive Control (MPC) on CENTAURO, which is a dual-arm quadrupedal legged manipulator. The focus is on achieving:
- **whole-body motion generation**: since the full kinematics of the robot is optimized within the MPC formulation. A dynamics model is as well considered (SRBD or full centroidal) permitting to generate dynamically feasible motions.
- **real-time performance**: since the formulations have been tested and achieve replanning frequencies at tens of Hz on the CENTAURO robot, which consists of 37 actuated joints.

<p float="left">
  <img src="https://github.com/IoannisDadiotis/ocs2_hhcm/assets/75118133/c2326436-ad7a-44ae-879c-329d0a114ded" alt="relax_example" width="760" height="280">
</p>

## Software Overview
The repository is based on the [ocs2](https://leggedrobotics.github.io/ocs2/index.html) library. Its structure follows the ocs2 examples (especially the legged robot example) and parts of the code have been taken from there. It is written in C++ and includes multiple `catkin` packages:
- `ocs2_centauro` that includes the necessary code for setting up the optimal control problem.
- `ocs2_centauro_ros` that includes all the ros-related wrappers/components for the above.
- `gazebo_ocs2_ros_interfaces` which implements an interface between the MPC and gazebo or the real hardware though ROS. The interface happens through *xbotcore* which is the middleware used for controlling CENTAURO.
- `ocs2_centauro_references` which is a (python-based) package for sending task-related references once the mpc is up and running (e.g. a desired reference trajectory). For now, this is limited for setting up the tasks used in our corresponding publication.

## Dependencies & System Requirements
Using the repository requires installing:
- ocs2 library (e.g. ocs_*) and their dependencies, e.g. Eigen, pinocchio, hpp-fcl. This repo is based on [this fork](https://github.com/IoannisDadiotis/ocs2).
- the package `xbot_msgs` and a xbot2 installation since this is the robotics middleware used for our robot (not necessary for visualization only in RViz). 
- the urdf model of CENTAURO that is considered in the MPC and can be found [here](https://github.com/IoannisDadiotis/ocs2_robotic_assets). Additionally, the repo [`iit-centauro-ros-pkg`](https://github.com/ADVRHumanoids/iit-centauro-ros-pkg) may be needed since it contains the meshes of the robot for visualization.
- the [`base_estimation`](https://github.com/ADVRHumanoids/base_estimation) package (branch `ioannis_centauro`) for estimating the floating base pose and twist of CENTAURO or any state estimation module that you prefer (not necessary for visualization only in RViz).

The code is tested in Ubuntu 20.04 with ros noetic. Ubuntu 18.04 and ros melodic has been tested as well in the past.

## Building the Code
First you need to install [this fork](https://github.com/IoannisDadiotis/ocs2) of the ocs2 library following the guidelines [here](https://leggedrobotics.github.io/ocs2/installation.html). After this, the packages of the current repository can be git cloned inside a catkin workspace and built with `catkin build`.

## Setting up the Optimal Control Problem (OCP)
The OCP is partially defined in the `ocs2_centauro` package, i.e. the state and control input vectors as well as the considered kinematics and dynamics. However the user can modify the configuration `*.info` files in `ocs2_centauro/config` for tuning the OCP formulation without the need to rebuild the packages each time a parameter changes. Through this configuration file most of the included constraints/cost terms can be activated/deactivated and/or tuned. Solver-related options can be as well tuned/selected through the `.info` files.

The implemented formulation includes the full kinematics and a user selected dynamics model (SRBD of full centroidal). The implementation solves a complex OCP with state vector of dimension 49 and control input dimension 55. The state includes the centroidal momentum, the floating base pose and the joint position of all 37 actuated joints of the robot (both lower and upper body). The control input vector includes the force at the contact points (4 legs and 2 arms) and the joint velocities of all the actuated joints.

This repository includes the implementation of various costs and constraints that have been tested on the real robot (self-collision avoidance, contact-related constraints, joint limits etc.). Various task specifications can be imposed (e.g. gait patterns, arm EE pose tracking) as hard constraints or through cost terms.

## Running the examples
After having installed everything you can just run the following launch file.

`roslaunch ocs2_centauro_ros centauro_ddp.launch` or
`roslaunch ocs2_centauro_ros centauro_ddp.launch task:=locomotion`

After waiting some time for generating the necessary files with automatic differentation you should be able to see the robot on RViz. The terminal argument `task` can be defined so as to include a specific task-related `.info` file (some of which are already there, e.g. `centauro_locomotion.info`, `centauro_task_soft.info`, `centauro_task.info` etc.). If the robot has to be run only in rviz (using the model in the MPC as rollout instance) set `xbotCoreRunning false` in the task file. If you want to deploy the MPC with the robot in gazebo or the real robot then first make sure the robot (real or simulated) is on a nominal configuration (e.g. standing in homing configuration) before launching the MPC. In the taskfile set `xbotCoreRunning true` and `xbotCoreFeedback true` (for sending the mpc output to xbotcore and receiving the state observation, respectively).
<p float="left">
  <img src="https://github.com/IoannisDadiotis/ocs2_hhcm/assets/75118133/a42ab881-14ad-4032-b5e8-9baf73fa3094" alt="relax_example" width="350" height="250">
  <img src="https://github.com/ADVRHumanoids/wb_mpc_centauro/assets/75118133/ca9f10b3-11c2-4ac2-a453-a9bfdc9f1c7a" alt="relax_example" width="350" height="250">  
</p>

## Deployment with a full physics simulation or the real robot in the loop
Deployment on the Gazebo simulator as well as on the real robot has been achieved using the motion planning and control framework shown below. The main components of this framework are:
- the whole-body MPC implemented in this repository
- a low-lever reference generator which sends torque feedforward, joint position and velocity references to the low-level joint impedance controllers. This is done by sampling the MPC policy and mapping it to torque feedforward through RNEA inverse dynamics.
- a state estimation module that updates the MPC with the most recent state observation.

**No instantaneous WBC** is used at the low-level since the MPC optimizes whole-body motions.

<img src="https://github.com/IoannisDadiotis/ocs2_hhcm/assets/75118133/5c07d1e4-dab8-49a3-852f-6334b7c6a0c9" alt="relax_example" width="650" height="150">

## LICENSE & ACKNOWLEDGEMENTS
The repo includes the BSD 3 license. The repo has been built following the structure of the examples of ocs2 library. Credits to the ocs2 authors are acknowledged in files that have been taken from there with little or no modifications.
