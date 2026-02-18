# S2911476 Coursework 2 Repo

This repo contains my submission for the Introduction to Mobile Robotics Coursework 2 assignment for the University of Edinburgh. This assignment involved the design and implementation of a particle filter to localise a simulated robot in a pre-generated map.

## Acknowledgement

This repo was forked from https://github.com/LCCZK/MOB_Coursework_2.

## Pre-requisites

- ROS2 Jazzy
- Ubuntu 24.04
    - My development was performed in WSL
- nav2_amcl, nav2_lifecycle_manager, slam_toolbox, rviz2, teleop_twist_keyboard
    ```
    sudo apt update
    sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
    ```

## Quick Start

1. Clone the Github repository 
    ```bash
    git clone https://github.com/wentuoc/MOB_Coursework_2.git
    ```

2. Go the the root directory of the repo and update all submodules.
    ```bash
    cd MOB_Coursework_2
    git submodule update --init --recursive
    ```
3. Source the ROS2 installation and set the ROS_LOCALHOST_ONLY environment variable
    ```bash
    echo "source /opt/ros/jazzy/setup.sh" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
    ```

4. Build the ROS workspace with `colcon`, 
    - Run from the root directory of the repo; `--symlink-install` is not supported for this workspace
    ```bash
    colcon build
    ```

5. Source the workspace with 
    ```bash
    source install/setup.bash 
    ```

### Playing data from a rosbag
6. Play a rosbag
    ```bash
    cd src/pf_localisation/data/sim_data/
    ros2 bag play simpath1 --clock -p
    ```

7. In another terminal, launch the nodes (remember to source `setup.bash`)
    ```bash
    ros2 launch pf_localisation pf_rosbag.launch.py
    ``` 

8. Optionally, run the grapher in another terminal
    - This will graph out the positions of the `ground_truth_pose` (if published), `amcl_pose` (from nav2_amcl), and `estimated_pose` (from my particle filter)
    ```
    ros2 run pf_grapher pf_grapher
    ```

9. Draw the initial pose of the robot (facing right at the origin) using Rviz's '2D Pose Estimate'

10. Press spacebar on the rosbag terminal to play the rosbag, and watch the robot's estimated pose move around

### Teleop
6. To control the robot using the teleop keyboard instead, use
    ```
    ros2 launch pf_localisation pf_teleop.launch.py
    ```

7. A separate window will pop up and the robot can then be controlled using the keys shown on screen

## Implementation Details
The different implementations of the `estimate_pose` method in `pf.py` is archived in the tags:
- v1.0: Using a naive average to calculate the final pose
- v1.1: Using the max weight to calculate the final pose
- v2.1: Using a weighted average to calculate the final pose

Additionally, the following tags mark other extensions to the filter algorithm
- v2.0: Solving the 'kidnapped' robot problem through a regeneration of the particle cloud once the weights dip below a pre-defined threshold
- v3.0: Implementing a formula to adapt the number of particles in the particle cloud based on the average weight