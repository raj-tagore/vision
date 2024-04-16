#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import numpy as np


def random_pose():
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = ""  # any frame you're working with
    
    # [ 0.3522457  -0.23809611 -0.07129898] [ 0.6141156   0.04148111 -0.10255097]
    # [ 0.35290719 -0.23809863 -0.07130083]
    
    pose.pose.position.x = 0.15 #np.random.uniform(0, 0.5) #0.15
    pose.pose.position.y = -0.2 #np.random.uniform(-0.7, 0.7) #0.0
    pose.pose.position.z = 0.5
    
    rotx = np.pi
    roty = 0
    rotz = 0
    
    quaternion = tf.transformations.quaternion_from_euler(rotx, roty, rotz)
    
    trans_matrix = tf.transformations.quaternion_matrix(quaternion)
    
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    
    return pose


if __name__ == '__main__':
    rospy.init_node('move_bot', anonymous=True)
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            pub.publish(random_pose())
            rospy.loginfo("Published pose message to equilibrium_pose")
            rate.sleep()    
    except rospy.ROSInterruptException:
        pass
