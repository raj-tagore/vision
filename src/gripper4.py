#!/usr/bin/env python

import rospy
from franka_gripper.msg import _GraspActionGoal

def grasp():
    grasp = _GraspActionGoal()
    grasp.goal.width = 0.05
    grasp.goal.speed = 0.1
    grasp.goal.force = 60.0
    grasp.goal.epsilon.inner = 0.01
    grasp.goal.epsilon.outer = 0.01
    
    