# wb_mpc_centauro
Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped
<p align="center">
  <a href="https://arxiv.org/abs/2310.02907">paper</a> •
  <a href="https://youtu.be/8XIAtw4201o">video</a> •
  <a href="#Citingthework">bibTeX</a>
</p>
<p float="left">
  <img src="https://github.com/IoannisDadiotis/ocs2_hhcm/assets/75118133/c2326436-ad7a-44ae-879c-329d0a114ded" alt="relax_example" width="745" height="275">
</p>

## Introduction
This repo implements Model Predictive Control (MPC) on CENTAURO, which is a dual-arm quadrupedal legged manipulator. The focus is on achieving:
- **whole-body motion generation**: since the full kinematics of the robot is optimized within the MPC formulation. A dynamics model is as well considered (SRBD or full centroidal) permitting to generate dynamically feasible motions.
- **real-time performance**: since the formulations have been tested and achieve replanning frequencies at tens of Hz on the CENTAURO robot, which consists of 37 actuated joints.

## Software Overview
The repository is based on the [ocs2](https://leggedrobotics.github.io/ocs2/index.html) library. Its structure follows the ocs2 examples (especially the legged robot example) and parts of the code have been taken from there. It is written in C++ and includes multiple `catkin` packages:
- `ocs2_centauro` that includes the necessary code for setting up the optimal control problem.
- `ocs2_centauro_ros` that includes all the ros-related wrappers/components for the above.
- `gazebo_ocs2_ros_interfaces` which implements an interface between the MPC and gazebo or the real hardware though ROS. The interface happens through *xbotcore* which is the middleware used for controlling CENTAURO.
- `ocs2_centauro_references` which is a (python-based) package for sending task-related references once the mpc is up and running (e.g. a desired reference trajectory). For now, this is limited for setting up the tasks used in our corresponding publication.

## Installation & Building the Code
If you have one of the supported Ubuntu OS (18.04 or 20.04) and ros distributions (melodic/noetic), you can install the install everything locally. Otherwise, you can use the framework by running everything in a docker container.

### Local installation
- First create a catkin workspace and an `src/` folder inside the workspace. We are going to build the code in the workspace using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/quick_start.html).
- Clone [this fork](https://github.com/IoannisDadiotis/ocs2_robotic_assets) containing the assets, including CENTAURO robot, in the src folder.
- Clone [this fork](https://github.com/IoannisDadiotis/ocs2) of the ocs2 library and install following the guidelines [here](https://leggedrobotics.github.io/ocs2/installation.html). This library has many dependencies among which Eigen, pinocchio, hpp-fcl. Make sure that the code is built successfully with `catkin build`.
- Clone the current repo `wb_mpc_centauro`.
- Clone somewhere the repos containing the necessary models for visualizing CENTAURO, i.e. [iit-centauro-ros-pkg](https://github.com/ADVRHumanoids/iit-centauro-ros-pkg) (contains the meshes of the robot), [iit-dagana-ros-pkg](https://github.com/ADVRHumanoids/iit-dagana-ros-pkg) (meshes for the gripper), [realsense_gazebo_description](https://github.com/m-tartari/realsense_gazebo_description), [realsense_gazebo_plugin](https://github.com/m-tartari/realsense_gazebo_plugin), [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/) (meshes for the sensor). These repos do not have to be cloned in the catkin workspace (since they do not have to be built), but anywhere else. You just have to add their path to your `$ROS_PACKAGE_PATH` through `export ROS_PACKAGE_PATH=<path_to_repos_with_models>:$ROS_PACKAGE_PATH`
- (Optional for Gazebo/Hardware deployment) Clone [base_estimation](https://github.com/ADVRHumanoids/base_estimation) package (branch `ioannis_centauro`) or any state estimation module that you prefer for estimating the floating base pose and twist of the robot.
- (Optional for Gazebo/Hardware deployment) Install the xbot2 middleware used for our robot in simulation and hardware. You can install the free binary release of xbot2, available [here](https://advrhumanoids.github.io/xbot2/master/index.html).
- Finally, build everything by running `catkin build` in your catkin workspace folder.

Notice that `catkin build` commands can take a lot of time! Make sure to run this when you have other things to do in parallel or take a coffee!!!

### Docker-based installation and deployment
We provide the docker image needed to run everything inside a docker container using Ubuntu 20.04 and ros noetic. All the docker-related files are in the `.devcontainer` folder in this repo. To start with, run `xhost +` in a terminal on your host, so that all users have access to X.

**Deployment through terminal:** Inside the `.devcontainer` folder you can find three main scripts, `build.sh`, `run.sh`, `attach.sh`. Navigate to this folder and you can use them as follows from your terminal:

`./build.sh <your_preferred_image_name>` to build the docker image

`./run.sh <your_built_image_name> <your_preferred_container_name>` to run the image inside a docker container

`./attach.sh <you_running_container_name>` to attach a new terminal inside the running container

`./catkin_ws_build` to build the caktin workspace

**Deployment through VSCode:** If you prefer so, you can build and run the docker image using VSCode, instead. To this end, open the `wb_mpc_centauro` repo in VSCode, then click on the bottom left symbol "Open a remote window" and the select the option "Reopen in Container" (or simply press `ctl`+`Shift`+`p` and type "Reopen in Container"). VSCode will start building the image, run it in a container and then build the catkin workspace. This will take a significant amount of time! Grab a coffee or two!

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

## <a name="Citingthework"></a>Citing the work
This repo implements the whole-body MPC presented in the following publication: <br />
*I. Dadiotis, A. Laurenzi, N. Tsagarakis*
**"Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped" in 2023 IEEE-RAS International Conference on Humanoid Robots (Humanoids)**
```
@inproceedings{dadiotis2023whole,
  title={Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped},
  author={Dadiotis, Ioannis and Laurenzi, Arturo and Tsagarakis, Nikos},
  booktitle={2023 IEEE-RAS 22nd International Conference on Humanoid Robots (Humanoids)},
  pages={1--8},
  year={2023},
  organization={IEEE}
}
```

## LICENSE & ACKNOWLEDGEMENTS
The repo includes the BSD 3 license. The repo has been built following the structure of the examples of ocs2 library. Credits to the ocs2 authors are acknowledged in files that have been taken from there with little or no modifications.
