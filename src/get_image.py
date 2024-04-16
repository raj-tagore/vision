#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv_bridge
import cv2

def callback(data):
    
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except cv_bridge.CvBridgeError as e:
        print(e)

    # Display the converted image
    cv_image = cv_image[:, :460]
    cv2.imshow("RGB Image", cv_image)
    cv2.imwrite("../images/d_image1.jpg", cv_image)
    cv2.waitKey(3)

def main():
    rospy.init_node('get_image_node', anonymous=True)
    rospy.Subscriber("camera/rgb/image_raw", Image, callback)
    
    # Keep the program alive.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
