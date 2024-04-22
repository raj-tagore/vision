#! /usr/bin/env python

import rospy
from moveit_commander import RobotCommander, MoveGroupCommander

# Initialize the ROS node
rospy.init_node('move_bot_node')

# Create a RobotCommander object
robot = RobotCommander()

# Create a MoveGroupCommander object for the arm
arm_group = MoveGroupCommander('panda_arm')

# Set the planning time
arm_group.set_planning_time(5)

def move_to_pose(pose):
    # Set the target pose
    arm_group.set_pose_target(pose)

    # Plan and execute the trajectory
    arm_group.go(wait=True)
    
def move_to_joint_values(joint_values):
    # Set the target joint values
    arm_group.set_joint_value_target(joint_values)

    # Plan and execute the trajectory
    arm_group.go(wait=True)

if __name__ == '__main__':
    # Define the target pose
    target_pose = arm_group.get_current_pose().pose
    target_pose.position.x += 0.1
    target_pose.position.y += 0.1
    target_pose.position.z += 0.1

    # Move the end effector to the target pose
    move_to_pose(target_pose)