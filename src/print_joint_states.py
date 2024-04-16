#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import os

def joint_states_callback(msg):
    joint_names = msg.name
    joint_positions = msg.position
    joint_velocities = msg.velocity

    # Clear console
    os.system('cls' if os.name == 'nt' else 'clear')

    # Print joint states to console
    for name, position, velocity in zip(joint_names, joint_positions, joint_velocities):
        rospy.loginfo(f"Joint: {name}, Position: {position:.2f}")

if __name__ == '__main__':
    rospy.init_node('joint_states_subscriber', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    rospy.spin()
