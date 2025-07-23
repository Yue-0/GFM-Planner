## Introduction

__GFM-Planner__ is a LiDAR-based perception-aware trajectory planner that improves localization accuracy during navigation by enabling the robot to actively avoid areas with sparse geometric features.

## Paper

__[GFM-Planner: Perception-Aware Trajectory Planning with Geometric Feature Metric](http://arxiv.org/abs/2507.16233){target="_blank"}__. Accepted by __IROS 2025__.

Authors: [Yue Lin](https://github.com/Yue-0){target="_blank"}, [Xiaoxuan Zhang](https://github.com/Zhxx-R){target="_blank"}, Yang Liu, Dong Wang and Huchuan Lu.

## Video

https://github.com/user-attachments/assets/27312007-2d57-4e55-a649-f9b6526d7c70

## Quick Start

In Ubuntu20.04 & ROS-noetic:

__Step 1__: Clone this project and build.

```shell
git clone https://github.com/Yue-0/GFM-Planner.git
pip install trimesh numpy opencv-python
cd GFM-Planner
catkin_make
```

__Step 2__: Build the map for Gazebo simulation and metric evaluation, this would take a while.

```shell
source devel/setup.bash
roslaunch simulator build.launch
```

__Step 3__: Start the simulation environment.

```shell
roslaunch simulator simulation.launch
```

And launch the planner in another terminal:

```shell
roslaunch planner planning.launch
```

Use `2D Nav Goal` to specify the goal pose.
