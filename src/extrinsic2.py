#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import moveit_commander
import numpy as np
import tf

class Camera:
    def __init__(self):
        self.position = None
        self.orientation = None
        self.T_pub = rospy.Publisher('rgp2/camera_transform', Float32MultiArray, queue_size=10)
        
    def handle_pose(self, pose):
        
        self.position = pose[0]
        self.orientation = pose[1]
        
        quaternion = [
        self.orientation[0],
        self.orientation[1],
        self.orientation[2],
        self.orientation[3]
        ]

        # Convert quaternion to a transformation matrix
        transformation_matrix = tf.transformations.quaternion_matrix(quaternion)

        # Extract the position components of the pose and set them in the matrix
        transformation_matrix[0][3] = self.position[0]
        transformation_matrix[1][3] = self.position[1]
        transformation_matrix[2][3] = self.position[2]
        
        # rospy.loginfo("Transformation Matrix: %s", transformation_matrix.round(2))
        
        transformation_matrix_msg = Float32MultiArray(data=transformation_matrix.flatten().tolist())
        self.T_pub.publish(transformation_matrix_msg)

if __name__ == '__main__':

    rospy.init_node('rgp2_get_camera_pose')
    camera = Camera()
    
    # Create a tf listener
    listener = tf.TransformListener()
    
    # Wait for up to 4 seconds for the frame transformation to become available
    listener.waitForTransform("camera_depth_link", "panda_link0", rospy.Time(0), rospy.Duration(4.0))
    
    # Get the current time
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        
        time = rospy.Time(0)
        camera_pose = listener.lookupTransform("camera_depth_link", "panda_link0", time)
        camera.handle_pose(camera_pose)
        rate.sleep()

