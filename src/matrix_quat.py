#! /usr/bin/env python

import rospy
import tf

XorientationMatrix =    [[1, 0, 0, 0], 
                        [0, 1, 0, 0], 
                        [0, 0, 1, 0], 
                        [0, 0, 0, 1]]

YorientationMatrix =    [[0, -1, 0, 0], 
                        [1, 0, 0, 0], 
                        [0, 0, 1, 0], 
                        [0, 0, 0, 1]]

ZorientationMatrix = [[0, 0, -1, 0], 
                    [0, 1, 0, 0], 
                    [1, 0, 0, 0], 
                    [0, 0, 0, 1]]

Xquaternion = tf.transformations.quaternion_from_matrix(XorientationMatrix)
Yquaternion = tf.transformations.quaternion_from_matrix(YorientationMatrix)
Zquaternion = tf.transformations.quaternion_from_matrix(ZorientationMatrix)

print("Xquaternion: ", Xquaternion)
print("Yquaternion: ", Yquaternion)
print("Zquaternion: ", Zquaternion)