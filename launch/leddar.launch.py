from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess

# https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed.launch.py
# Also need to add config to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    leddar_stream = Node(
        package="leddar_can",
        executable="leddar_stream",
        parameters=[{'max_stream_flag': False},
                    {'rate_hz': 10},
                    {'min_amp_slope': 1.5},
                    {'min_amp_offset': 15.0}
        ]
    )
    
    ld.add_action(leddar_stream)
    
    return ld