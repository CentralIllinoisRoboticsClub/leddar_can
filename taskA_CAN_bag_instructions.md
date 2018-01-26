You can use the .bag file (public sharing of bag file pending) to test other nodes without the actual robot.

After getting the code from github (put it in ~/catkin_ws/src/), you will need to build
cd ~/catkin_ws
catkin_make
source ~/.bashrc

NOTE .bashrc is sourced every new terminal/tab. The key lines for ROS in .bashrc are:
```
source /opt/ros/kinetic/setup.bash
source /home/<user>/catkin_ws/devel/setup.bash
#(For slave ROS cpu)
# xxxxx can be seen in the output of roscore
ROS_MASTER_URI=http://<hostname>:xxxxx/
```

Run the following terminal commands (separate terminals/tabs for each, I use tmux):

`roscore`

`rosrun leddar_can leddar_stream`

`cd </path/to/bag/folder/>`
`rosbag play wheele_CAN_odom_scan4_20180102.bag --topics /received_messages /sent_messages`

To verify it is working, try
`rostopic list`
and look for /scan.
Then try
`rostopic echo /scan`

`rosrun rqt_graph rqt_graph`

`rosrun rviz rviz`

In rviz, you want to just observe the laser data in the laser frame or base_link frame.
Turn off Map if it is checked. Make sure LaserScan is checked and the topic is /scan.
If you have trouble getting rviz setup, backup your home/<user>/.rviz/default.rviz file and use the attached one from my setup.

Once this is working, and you understand the leddarCAN.cpp and leddar_can.py nodes, try to modify can2ros in the wheele repo.
The goal is to remove the use of the python can bus (socketcan) interface (currently self.bus).
We want to make use of the published /received_messages from socketcan_bridge_node.
All of the cmd, gyro, encoder, and battery data is published on /received_messages as can_msgs/Frame data types.
So you can test your new can2ros.py (go ahead and rename it something else, then just run rosrun can2ros_wheele can2ros_RENAMED.py)
You may need to do
sudo chmod +x /path/to/can2ros_RENAMED.py

For subscribing to /received_messages, you will need to have can_msgs,
So go ahead and:
`sudo apt install ros-kinetic-ros-canopen`

In python:
```
from can_msgs.msg import Frame

rospy.Subscriber('received_messages', Frame, self.can_callback, queue_size = 50)

def can_callback(self, frame_data):
  id = frame_data.id
  data = frame_data.data #8 bytes, not sure how python stores these exactly
```
If you are comfortable with the leddarCAN.cpp, you may just want to try the cpp approach for "can2ros" in parallel.
