#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gripper', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # print("============ Available Planning Groups:", robot.get_group_names())
    
    group_name = "panda_hand"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    
    while rospy.is_shutdown() is False:
        try:
            msg = rospy.wait_for_message("/rgp2/gripper", String, timeout=1)
            if msg.data == "open":
                group.set_named_target("open")
                plan = group.go(wait=True)
            elif msg.data == "close":
                group.set_named_target("close")
                plan = group.go(wait=True)
        except:
            pass