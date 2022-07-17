#!/usr/bin/python3

from platform import node

import rospy
import roslib
import time
import numpy as np
#roslib.load_manifest('cmd_dir_image')
import cv2
from std_msgs.msg import Int8MultiArray ,Int8
from sensor_msgs.msg import LaserScan

class detect_area_node:
    def __init__(self):
        self.range_0 = []
        self.range_45 =[]
        self.range_90 =[]
        self.range_135 =[]
        self.range_180 =[]
        self.num_0 =np.array([])
        self.num_45 =np.array([])
        self.num_90 =np.array([])
        self.num_135 =np.array([])
        self.num_180 =np.array([])
        self.cmd_dir_sub = rospy.Subscriber("/scan",LaserScan, self.scan_callback)
        #self.episode_num_sub = rospy.Subscriber("/episode_num",Int8, self.num_callback)
        #self.step_num = 100

    def scan_callback(self,data):
        self.range_0 = data.ranges[0]
        self.range_45 =data.ranges[45]
        self.range_90 =data.ranges[90]
        self.range_135 =data.ranges[135]
        self.range_180 =data.ranges[180]
        
        self.num_0 =np.array(data.ranges[0])
        self.num_45=np.array(self.ranges[45])
        self.num_90=np.array(self.ranges[90])
        self.num_135=np.array(self.ranges[135])
        self.num_180=np.array(self.ranges[180])


    def loop(self):
        print(self.num.shape)

if __name__ == '__main__':
    rospy.init_node('cmd_dir_img')
    hoge = detect_area_node()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        hoge.loop()
        r.sleep()