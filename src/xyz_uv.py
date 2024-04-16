#! /usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import tf
from std_msgs.msg import Float32MultiArray

def callback(data):
    K = [554.3827128226441, 0.0, 320.5, 0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    K = np.array(K).reshape(3, 4)
    pose = listener.lookupTransform("panda_link0", "camera_depth_link", rospy.Time(0))
    pos = pose[0]
    ori = pose[1]
    quat = [ori[0], ori[1], ori[2], ori[3]]
    tf_mtx = tf.transformations.quaternion_matrix(quat)
    tf_mtx[0][3] = pos[0]
    tf_mtx[1][3] = pos[1]
    tf_mtx[2][3] = pos[2]
    
    points = np.array(list(pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)))
    xyz_uv = np.zeros((480, 460, 3), dtype=np.float32)
    
    for point in points:
        xyz = np.array([point[0], point[1], point[2], 1]).T
        uv1 = np.dot(K, xyz) / xyz[2]
        if uv1[0] < 0 or uv1[0] >= 460 or uv1[1] < 0 or uv1[1] >= 480:
            continue
        xyz = np.dot(tf_mtx, xyz)
        xyz_uv[int(uv1[1]), int(uv1[0])] = [xyz[0], xyz[1], xyz[2]]
    
    pub.publish(Float32MultiArray(data=xyz_uv.flatten().tolist()))
    print("Published xyz_uv of size 480x460x3.")


if __name__ == "__main__":
    
    rospy.init_node("xyz_uv_node")
    
    listener = tf.TransformListener()
    listener.waitForTransform("panda_link0", "camera_depth_link", rospy.Time(0), rospy.Duration(4.0))
    
    sub = rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
    pub = rospy.Publisher("/rgp2/xyz_uv", Float32MultiArray, queue_size=2)
    rospy.spin()