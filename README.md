# Lidar project
## Project Overview
This project integrates a simulated LiDAR sensor in CoppeliaSim with ROS 2 for real-time SLAM (Simultaneous Localization and Mapping) and navigation in RViz. The sensor data is processed and published using a Python node, which broadcasts transforms and handles obstacle avoidance by referencing an occupancy grid. The generated map is then used to plan the robot's route using the D* Lite algorithm, targeting an algorithm selected based on its distances from the robot and the starting point, simulating a limited amount of energy.

## Prerequisites
Ensure you have the following installed:
- [CoppeliaSim](https://www.coppeliarobotics.com/) (with a compatible version for your system)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- Python 3.12 (compatible with ROS 2 packages)

## CoppeliaSim Scene File
- **LiDAR Scene File (.ttt)**: Required CoppeliaSim scene with LiDAR object configured. The `LidarPublisher` node retrieves and processes data from this scene. `sick.ttt` was the scene used during initial testing, containing the track space from last year. `sick_updated.ttt` is this year's scene, imported from a scan of the track used.

## How to install Google Cartographer
  ```
  sudo apt install ros-jazzy-cartographer
  sudo apt install ros-jazzy-cartographer-ros
  sudo apt install ros-jazzy-cartographer-rviz
  ```

## Environment and packages
This repository is intended to be used with a virtual Python environment installed in the ROS workspace (ros2_ws/venv). This environment should contain all required packages. `setup_env.sh` activates the Python and ROS environments. You may find out what packages are missing by trying to run the code and then reading the error message (it should say something like "ModuleError: package x not found"). You can install them as usual (`pip install x`) as long as you have sourced the virtual Python environment.

## Main launch file
You may compile, source and run the code in one line with the following command:
```
  colcon build; source install/setup.bash; ros2 launch te3003b_rover lidar_slam_map.launch.py
``` 
