# leddar_can

ROS package for Leddar M16, 45 deg using CAN communication

Publishes sensor_msgs LaserScan /scan

PENDING: parameters for choosing laser scan rate, laser w.r.t. baselink, etc.

## Usage

CAN bitrate 500000
```
sudo ip link set can0 up type can bitrate 500000
cansend can0 00000740#02
sudo ip link set can0 down
rosrun leddar_can leddar_can.py
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 /baselink /laser 10
