#!/usr/bin/env python

import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal
from franka_gripper.msg import MoveAction, MoveGoal


def move():
    move = MoveGoal()
    
    move.width = 0.08 # max width is 0.08
    move.speed = 0.1
    
    client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
    client.wait_for_server()
    client.send_goal(move)
    client.wait_for_result()
    result = client.get_result()
    print(result)
    
    return result

def grasp():
    client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
    client.wait_for_server()

    goal = GraspGoal()
    goal.width = 0.05
    goal.epsilon.inner = 0.1
    goal.epsilon.outer = 0.1
    goal.force = 50
    goal.speed = 0.1

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the action to complete
    client.wait_for_result()
    
    result = client.get_result()

    print(result)

if __name__ == '__main__':
    rospy.init_node('gripper', anonymous=True)
    try:
        grasp()
    except rospy.ROSInterruptException:
        pass