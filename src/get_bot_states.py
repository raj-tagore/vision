#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState

class EndEffectorPose:
    def __init__(self):
        rospy.init_node('end_effector_pose_calculator', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
    def joint_state_callback(self, msg):
        # Ensure the joint names match the order in MoveIt
        joint_positions = [msg.position[msg.name.index(j)] for j in self.group.get_active_joints()]
        self.group.set_joint_value_target(joint_positions)
        
        # # Get end effector pose
        # ee_pose = self.group.get_current_pose()
        # rospy.loginfo("End Effector Position: %s", ee_pose.pose.position)
        # rospy.loginfo("End Effector Orientation: %s", ee_pose.pose.orientation)
        
        # ee_link = self.group.get_end_effector_link()
        # rospy.loginfo("End Effector Link: %s", ee_link)
        
        # group_names = self.robot.get_group_names()
        # rospy.loginfo("Available Planning Groups: %s", group_names)
        
        # state = self.robot.get_current_state()
        # rospy.loginfo("Current State: %s", state)
        
        ee_pose = self.group.get_current_pose(end_effector_link="camera_link")
        rospy.loginfo("Position: %s", ee_pose.pose.position)
        rospy.loginfo("Orientation: %s", ee_pose.pose.orientation)

if __name__ == '__main__':
    try:
        eep = EndEffectorPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
