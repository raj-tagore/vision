#!/usr/bin/env python

import rospy
import moveit_commander

def operate_gripper(grip):
        """
        Operate the gripper: Open or close based on the open_close flag.
        True for open, False for close.
        """
        gripper_group_name = "panda_hand"
        gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
        
        # Set the gripper force (example value, adjust based on your robot)
        gripper_group.set_max_force(10.0)
        
        if grip:
            # Open the gripper (example values, adjust based on your robot)
            gripper_group.set_joint_value_target([0.00, 0.00])
        else:
            # Close the gripper (example values, adjust based on your robot)
            gripper_group.set_joint_value_target([0.04, 0.04])
        
        gripper_group.go(wait=False)
    

if __name__ == '__main__':
    
    pose_list = [[0.6141156, -0.04148111, 0.10255097], 
                 [0.6141156, -0.04148111, -0.10255097],
                 [0.6141156, -0.04148111, 0.10255097],] 
    
    rospy.init_node('gripper', anonymous=True)
    
    operate_gripper(True)
    
    
    
    
    
    