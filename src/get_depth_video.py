#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg)
    cv_image = cv_image[:, :460]
    cv2.imshow("Video Feed", cv_image)
    cv2.waitKey(1)
    
if __name__ == "__main__":
    rospy.init_node("video_feed_node")
    rospy.Subscriber("/camera/depth/image_raw", Image, image_callback)
    rospy.spin()