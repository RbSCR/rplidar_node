# rplidar_node

A ROS2 node for SLAMTEC LIDAR.

Provides the node 'rplidar_node' to use the RPLIDAR and produce LaserScan messages.

Uses the 'rplidar_sdk_ros2' package, which provides the public SDK of RPLIDAR products in C++ as a ROS package.

## Publishes

- /scan  (sensor_msgs/msg/LaserScan)

## Parameters

- channel_type : Specifying channel type of the lidar - (string)
- tcp_ip : Specifying tcp ip to the connected lidar - (string)
- tcp_port : Specifying tcp port to the connected lidar - (int)
- udp_ip : Specifying udp ip to the connected lidar - (string)
- udp_port : Specifying udp port to the connected lidar - (int)
- serial_port : Specifying usb port to the connected lidar - (string)
- serial_baudrate : Specifying usb port baudrate to the connected lidar - (int)
- frame_id : Specifying frame_id of lidar - (string)
- inverted : Specifying whether or not to invert scan data - (bool)
- angle_compensate : "Specifying whether or not to enable angle_compensate of the scan data - (bool)
- flip_x_axis : Specifying if the X axis should be flipped - (bool)
- auto_standby : Specify wether to switch to standby mode. In standby start and stop services are disabled - (bool)
- topic_name : Specifying the topic name of the LaserScan message - (string)
- scan_mode : Specifying scan mode of the lidar - (string)
- scan_frequency : Specifying scan frequency of the lidar - (float)

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
