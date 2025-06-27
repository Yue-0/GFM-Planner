## Introduction

__GFM-Planner__ is a LiDAR-based perception-aware trajectory planner that improves localization accuracy during navigation by enabling the robot to actively avoid areas with sparse geometric features.

## Paper

__GFM-Planner: Perception-Aware Trajectory Planning with Geometric Feature Metric__. Accepted by __IROS 2025__. The full paper will be published soon.

Authors: [Yue Lin](https://github.com/Yue-0), [Xiaoxuan Zhang](https://github.com/Zhxx-R), Yang Liu, Dong Wang and Huchuan Lu.

## Video

https://github.com/user-attachments/assets/859af1e7-2bbc-4ee0-b38c-bcda333494cf

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

## Acknowledgements

We use [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) to solve numerical optimization problems.
