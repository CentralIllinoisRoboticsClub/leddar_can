# leddar_can

ROS package for Leddar M16, 45 deg using CAN communication

Publishes sensor_msgs LaserScan /scan

PENDING: parameters for choosing laser scan rate, laser w.r.t. baselink, etc.

Dependencies:
`sudo apt install ros-kinetic-ros-canopen`
(socketcan_bridge and can_msgs)

## Usage

CAN bitrate 500000
```
sudo ip link set can0 up type can bitrate 500000
```
For the python node:
```
cansend can0 00000740#02
rosrun leddar_can leddar_can.py
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 /baselink /laser 10
```

OR, For the cpp node:
(You can set laser scan rate in leddar.launch)
```
rosrun socketcan_bridge socketcan_bridge_node
roslaunch leddar_can leddar.launch
```

When done with CAN:
```
sudo ip link set can0 down
```
