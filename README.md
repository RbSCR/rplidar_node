# rplidar_node

A ROS2 node for SLAMTEC LIDAR.

Provides the node 'rplidar_node' to use the RPLIDAR and produce LaserScan messages.

Uses the 'rplidar_sdk_ros2' package, which provides the public SDK of RPLIDAR products in C++ as a ROS package.

## Run rplidar_node

```bash
ros2 run rplidar_node rplidar_node [parameters ...]
```

Or with the launch-file (for a RPLIDAR C1):

```bash
ros2 launch rplidar_node rplidar_c1_launch.py
```

## View output in rviz

To view the output in RVIZ:

- use the rplidar_utilities package
- or start RVIZ with 'ros2 run rviz rviz'

---

![C++17](https://img.shields.io/badge/C++-17-green)
![License](https://img.shields.io/badge/License-BSD--2--Clause-orange)

Tested with:

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![RPLIDAR C1](https://img.shields.io/badge/RPLIDAR--C1-green)

---
