#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv_bridge
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf.transformations

def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except cv_bridge.CvBridgeError as e:
        print(e)     
    with open('n.txt', 'r+') as file:
        n = file.read()
        cv2.imwrite("../images/image"+n+".jpg", cv_image)
        n=str(int(n)+1)
        file.seek(0)
        file.write(n)
    
def random_pose():
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = ""  # any frame you're working with
    pose.pose.position.x = np.random.uniform(0, 0.5)
    pose.pose.position.y = np.random.uniform(-0.7, 0.7)
    pose.pose.position.z = 0.5
    
    rotx = np.pi
    roty = 0
    rotz = 0
    
    quaternion = tf.transformations.quaternion_from_euler(rotx, roty, rotz)
    
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    
    return pose
    
def main():
    rospy.init_node('make_dataset_node', anonymous=True)
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
            pub.publish(random_pose())
            rate.sleep()    
            sub = rospy.Subscriber("camera/rgb/image_raw", Image, callback)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()