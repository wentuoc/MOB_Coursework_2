# Introduction to Mobile Robotic - Coursework 1
The objective of this exercise is for you to implement, analyze, and understand the working of a particle filter localisation algorithm. You will be working in simulation, using the `Stage` environment and `RViz 2`.

## 1. Setup your workspace
**\*The following code just a demostration, it is recommend that you fork the given repo.**

Clone the Github repository 
```
git clone git@github.com:LCCZK/MOB_Coursework_1.git
```

Go the the root directory of the repo and update all submodules.
```
cd MOB_Coursework_1
git submodule update --init --recursive
```
Install dependencies with `rosdep`. **\*run form the root directory of the repo**.
```
rosdep install --from-paths src
rosdep update
```
## 2. Working with ROS
Build your ROS workspace with `colcon`, **\*run form the root directory of the repo**.
```
colcon build --symlink-install
```
## 3. Your tasks
## 4. Test your implementation with simulation