#!/usr/bin/env python

import rospy
import tf
import numpy as np

if __name__ == "__main__":
    
    coordinate_list = [[0.263, -0.225, 0.554], 
                       [0.219, -0.022, 0.554],
                       [0.073, 0.130, 0.634],
                       [0.680, -0.16, 0.609],
                       [0.433, 0.041, 0.64], 
                       [0.434, -0.33, 0.64]]
    
    
    rospy.init_node('convert_coords', anonymous=True)
    listener = tf.TransformListener()
    listener.waitForTransform("panda_link0", "world", rospy.Time(0), rospy.Duration(4.0))
    
    pose = listener.lookupTransform("panda_link0", "world", rospy.Time(0))
    pos = pose[0]
    ori = pose[1]
    quat = [ori[0], ori[1], ori[2], ori[3]]
    tf_mtx = tf.transformations.quaternion_matrix(quat)
    tf_mtx[0][3] = pos[0]
    tf_mtx[1][3] = pos[1]
    tf_mtx[2][3] = pos[2]
    
    for point in coordinate_list:
        point = np.array(point + [1])
        new_point = np.dot(tf_mtx, point)
        print(new_point[:3])