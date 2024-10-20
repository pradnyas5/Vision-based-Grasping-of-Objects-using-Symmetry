# Vision-based-Grasping-of-Objects-using-Symmetry

## 1. Introduction/Abstract
The "Grasping Assuming Symmetry" project leverages the nature of symmetry in many objects. Our approach uses a depth camera to capture real-time 3D representations of the environment, focusing on object symmetries to optimize grasping points. This integration of depth-sensing technology and algorithmic innovation offers a promising advancement in robotic manipulation, ensuring more stable and efficient interactions with many objects in real-world scenarios.

## 2. Implementation   

Before implementing, source ROS 2 workspace and follow the steps to build the code:

### Terminal 1

Launch simulation environment.

```bash
ros2 launch vbm_project_env simulation.launch.py
```

### Terminal 2

Run object sampling ros2 node.

```bash
ros2 run vbm_project_env object_sample.cpp
```

### Terminal 3

Observe the point cloud by adding topics.

```bash
rviz2
```
