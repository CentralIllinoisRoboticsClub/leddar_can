#!/usr/bin/env python

import rospy, math
import numpy as np

from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan

import time
import can
import sys
import binascii

class LeddarCAN():
    def __init__(self):
        rospy.init_node('leddar_scan')
        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
        print('rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 /base_link /laser 10')
        
        #self.prev_time = rospy.Time.now()
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')
        # http://python-can.readthedocs.io/en/latest/interfaces/socketcan.html
        
        self.scan = LaserScan()
        self.scan.header.stamp = 0
        self.scan.header.frame_id = 'laser'
        self.scan.angle_min = -0.3925 #-22.5*3.14/180.0
        self.scan.angle_max = 0.392 #22.5*3.14/180.0
        self.scan.angle_increment = 0.0523 #3.0*3.14/180.0
        self.scan.range_min = 0.1
        self.scan.range_max = 50.0
        self.scan.ranges = [0] * 16 #
        #self.scan.intensities = [1] * 16 #Not used yet, for amplitude data
        
    def get_leddar_scan(self):
        msg = self.bus.recv(0.0)
        
        wait_count = 0
        while(msg != None and wait_count < 200):
            wait_count = wait_count + 1
            if(msg.arbitration_id == 0x00000751): #Leddar beginning of scan, # detections
                #print msg.arbitration_id, convertCAN(msg.data, 2, 2)
                #read just the first byte (only byte) for num detections
                #NOTE, int(xx,8) is invalid. Using 16 still gives us the desired 8 bit value
                num_detections = int(binascii.hexlify(msg.data[0:1]),16) #must specify 0:1, not 0
                #if(num_detections > 16): # I actually observed up to 20 detections, angles get repeated
                #    num_detections = 16
                self.read_detections(num_detections)
                break
                
            msg = self.bus.recv(0.0) #Needed for while loop
        
    def read_detections(self, nDetections):
        current_time = rospy.Time.now()
        self.scan.header.stamp = current_time
        self.scan.ranges = [0] * 16
        det_count = 0
        
        msg = self.bus.recv(0.0)
        while(msg != None and det_count < nDetections):
            if(msg.arbitration_id == 0x00000750):
                for k in range(2): #0, 1
                    det_count = det_count + 1
                    if(det_count <= nDetections):
                        info = msg.data[0+4*k:4+8*k] #0:4, 4:8 means 0 thru 3, 4 thru 7
                        laser_num = int(binascii.hexlify(info[3:4]),16)/16 # shift right 4 bits
                        dist_byte1 = int(binascii.hexlify(info[1:2]),16)
                        dist_byte2 = int(binascii.hexlify(info[0:1]),16)
                        dist_cm = dist_byte1*256 + dist_byte2 # Need a better way for little endian in python
                        #int.from_bytes(byte_string, byteorder='little')
                        self.scan.ranges[laser_num] = dist_cm/100.0 #ros wants meters
            elif(msg.arbitration_id == 0x00000751):
                break
                
            msg = self.bus.recv(0.0)
        
        if(det_count > 0):
            self.scan_pub.publish(self.scan)

if __name__ == '__main__':
    try:
        leddar_can = LeddarCAN()
        print("Starting Leddar CAN")

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            leddar_can.get_leddar_scan()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass

            
