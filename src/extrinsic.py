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
        # intrinsic_matrix = [554.3827128226441, 0.0, 320.5, 0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        # self.intrinsic_matrix = np.array(intrinsic_matrix).reshape(3, 4)
        # projection_matrix = [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        # self.projection_matrix = np.array(projection_matrix).reshape(3, 4)
        # self.P_pub = rospy.Publisher('rgp2/projection_matrix', Float32MultiArray, queue_size=10)
        self.T_pub = rospy.Publisher('rgp2/camera_transform', Float32MultiArray, queue_size=10)
        
    def handle_pose(self, pose):
        self.position = pose.pose.position
        self.orientation = pose.pose.orientation
        
        quaternion = (
        self.orientation.x,
        self.orientation.y,
        self.orientation.z,
        self.orientation.w
        )

        # Convert quaternion to a transformation matrix
        transformation_matrix = tf.transformations.quaternion_matrix(quaternion)

        # Extract the position components of the pose and set them in the matrix
        transformation_matrix[0][3] = self.position.x
        transformation_matrix[1][3] = self.position.y
        transformation_matrix[2][3] = self.position.z
        
        # rospy.loginfo("Transformation Matrix: %s", transformation_matrix.round(2))
        
        transformation_matrix_msg = Float32MultiArray(data=transformation_matrix.flatten().tolist())
        self.T_pub.publish(transformation_matrix_msg)
        
        # Compute the projection matrix
        # self.projection_matrix = np.dot(self.intrinsic_matrix, transformation_matrix)
        # projection_matrix_msg = Float32MultiArray(data=self.projection_matrix.flatten().tolist())
        # self.P_pub.publish(projection_matrix_msg)
        # Round each value in the projection matrix to 2 decimals and display it
        # rospy.loginfo("Projection Matrix: %s", self.projection_matrix.round())

if __name__ == '__main__':

    group = moveit_commander.MoveGroupCommander("panda_arm")
    rospy.init_node('rgp2_camera')
    camera = Camera()
    rospy.loginfo(group.get_planning_frame())
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        camera_pose = group.get_current_pose(end_effector_link="camera_depth_link")
        camera.handle_pose(camera_pose)
        rate.sleep()
        