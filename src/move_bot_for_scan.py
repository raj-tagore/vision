#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def makePose(coordinates):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "panda_link0" 
    
    pose.pose.position.x = coordinates[0]
    pose.pose.position.y = coordinates[1]
    pose.pose.position.z = 0.5
    
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    
    return pose

def publishPose(pose):
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    time = rospy.Time.now()
    while not rospy.is_shutdown():
        pub.publish(pose)
        if rospy.Time.now() - time > rospy.Duration(20):
            break
        rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('move_bot', anonymous=True)
    coordinatesList = [[0.00, -0.40], [0.00, 0.00], [0.00, 0.30], 
                       [0.25, -0.40], [0.25, 0.00], [0.25, 0.30], 
                       [0.45, -0.40], [0.45, 0.00], [0.45, 0.30]]
    
    try:
        for coordinate in coordinatesList:
            pose = makePose(coordinate)
            publishPose(pose)
        
    except rospy.ROSInterruptException:
        pass
