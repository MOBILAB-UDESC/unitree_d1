# UNITREE D1

<p align="center">
  <img src="https://raw.githubusercontent.com/MOBILAB-UDESC/unitree_d1/main/doc/resources/unitree_d1.png" alt="unitree_d1" width="180"/>
</p>

## Description
ROS 2 description and simulation package for the **Unitree D1** manipulator with MoveIt 2 integration.

**Note:** This package is a submodule of the [arms](https://github.com/MOBILAB-UDESC/arms.git) meta-package.

## ROS 2 info
<p align="center">
|Ubuntu|ROS 2 Distro|Gazebo Version|
|:----:|:---------------:|:------------:|
|[24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)|[Jazzy](https://docs.ros.org/en/jazzy/index.html)|[Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)|
</p>

## Cloning and building
``` cli
mkdir ~/arms_ws && cd ~/arms_ws
git clone https://github.com/MOBILAB-UDESC/arms.git src
cd src
git submodule update --init unitree_d1 d1_2f
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Configuration
Compatible gripper:
- **d1_2f** (default).

## Simulation
#### Launch gazebo world
``` cli
ros2 launch arms_bringup arm_world_launch.py world_name:=playground
```

#### Spawn the robot
``` cli
ros2 launch arms_bringup arm.launch.py use_sim_time:=true arm:=unitree_d1 gripper:=d1_2f use_camera:=false rviz:=false
```

#### Launch moveit
``` cli
ros2 launch arms_bringup arm_moveit_launch.py use_sim_time:=true arm:=unitree_d1 gripper:=d1_2f rviz:=true
```