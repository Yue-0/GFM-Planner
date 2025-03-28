## Introduction

__GFM-Planner__ is a LiDAR-based perception-aware trajectory planner that improves localization accuracy during navigation by enabling the robot to actively avoid areas with sparse geometric features.

<!-- ## Paper

GFM-Planner: Perception-Aware Trajectory Planning with Geometric Feature Metric (IROS 2025 submission). -->

## Video

https://github.com/user-attachments/assets/859af1e7-2bbc-4ee0-b38c-bcda333494cf

## Quick Start

This project needs to run under Ubuntu20.04 & ROS-noetic.

```shell
git clone https://github.com/Yue-0/GFM-Planner.git
pip install trimesh numpy opencv-python
cd GFM-Planner
catkin_make
```

Build the map for Gazebo simulation and metric evaluation:

```shell
source devel/setup.bash
roslaunch simulator build.launch
```

Start the simulation environment:

```shell
roslaunch simulator simulation.launch
```

Launch the planner in another terminal:

```shell
roslaunch planner planning.launch
```

Use `2D Nav Goal` to specify the goal pose.

## Acknowledgements

We use [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) to solve numerical optimization problems.
