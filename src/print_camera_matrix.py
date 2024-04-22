#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
import os

def camera_info_callback(msg):
    os.system('cls' if os.name == 'nt' else 'clear')
    out = msg.P
    rospy.loginfo(out)
    
if __name__ == '__main__':
    rospy.init_node('camera_info_subscriber')
    rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, camera_info_callback)
    rospy.spin()