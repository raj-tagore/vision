#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from franka_gripper.msg import GraspAction, GraspGoal
from franka_gripper.msg import MoveAction, MoveGoal
import actionlib

class SceneObject:
    def __init__(self, ID, name, position, axis):
        self.ID = ID
        self.name = name
        self.position = position
        self.axis = axis

    def preGrasp(self):
        # move to pre-grasp position
        targetPose = PoseStamped()
        targetPose.pose.position.x = self.position[0] + 0.1
        targetPose.pose.position.y = self.position[1]
        targetPose.pose.position.z = (self.position[2] - 0.615) + 0.15
        
        if self.axis == 'x':
            targetPose.pose.orientation.x = 1.0
            targetPose.pose.orientation.y = 0.0
            targetPose.pose.orientation.z = 0.0
            
        elif self.axis == 'y':
            targetPose.pose.orientation.x = 0.71
            if self.position[1] > 0:
                targetPose.pose.orientation.y = 0.71
            else:
                targetPose.pose.orientation.y = -0.71
            targetPose.pose.orientation.z = 0.0
            targetPose.pose.orientation.w = 0.0
            
        else:
            targetPose.pose.orientation.x = 0.71
            targetPose.pose.orientation.y = 0.0
            targetPose.pose.orientation.z = 0.71
            
        self.executeForTime(targetPose, 6.9)
        return
    
    def open(self):
        move = MoveGoal()
        move.width = 0.08
        move.speed = 0.5
        client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
        client.wait_for_server()
        client.send_goal(move)
        client.wait_for_result()
        result = client.get_result()
        return result
    
    def moveToGrasp(self):
        # move to grasp position
        targetPose = PoseStamped()
        targetPose.pose.position.x = self.position[0] + 0.1
        targetPose.pose.position.y = self.position[1]
        targetPose.pose.position.z = self.position[2] - 0.629
        
        self.executeForTime(targetPose, 3)
        return
            
    def grasp(self):
        client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
        client.wait_for_server()
        goal = GraspGoal()
        if self.name == '1.0' or self.name == '0.0':
            goal.width = 0.05
        else:
            goal.width = 0.02
        goal.epsilon.inner = 0.00
        goal.epsilon.outer = 0.03
        goal.force = 40
        goal.speed = 0.5
        
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(6.9))
        result = client.get_result()
        print(result)
        return result
        
    def postGrasp(self):
        # move to bin
        targetPose = PoseStamped()
        targetPose.pose.position.x = -0.4
        if self.position[1] > 0:
            targetPose.pose.position.y = 0.5
        else:
            targetPose.pose.position.y = -0.5
        targetPose.pose.position.z = 0.3
        
        targetPose.pose.orientation.x = 1.0
        targetPose.pose.orientation.y = 0.0
        targetPose.pose.orientation.z = 0.0
        
        self.executeForTime(targetPose, 3)
    
    def pickPlacePipeline(self):
        self.open()
        self.preGrasp()
        self.moveToGrasp()
        self.grasp()
        self.preGrasp()
        self.postGrasp()
        return
    
    def executeForTime(self, targetPose, delay):
        # execute a move command for a specific time and then return
        time = rospy.Time.now()
        rate = rospy.Rate(10)
        while rospy.Time.now() - time < rospy.Duration(delay):
            pub.publish(targetPose)
            rate.sleep()
        return
    
    def executeTillIdle(self, targetPose):
        global botIsIdle
        botIsIdle = False
        # execute a move command till the robot is idle
        rate = rospy.Rate(10)
        while not botIsIdle:
            pub.publish(targetPose)
            rate.sleep()
        return
            

def pacer(data):
    global botIsIdle
    magnitude = np.linalg.norm(data.velocity)
    if magnitude < 0.02:
        botIsIdle = True    

def parseObjData(name, pose):
    splitName = name.split("_")
    objId = splitName[0]
    objType = splitName[1]
    objAxis = splitName[2]
    objPosition = [pose.position.x, pose.position.y, pose.position.z]
    sceneObject = SceneObject(objId, objType, objPosition, objAxis)
    return sceneObject

def homeRobot():
    homePose = PoseStamped()
    homePose.pose.position.x = 0.25
    homePose.pose.position.y = 0.0
    homePose.pose.position.z = 0.5
    homePose.pose.orientation.x = 1.0
    homePose.pose.orientation.y = 0.0
    homePose.pose.orientation.z = 0.0
    homePose.pose.orientation.w = 0.0
    time = rospy.Time.now()
    while rospy.Time.now() - time < rospy.Duration(6.9):
        pub.publish(homePose)
    

if __name__ == '__main__':
    rospy.init_node('pick_drop', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    
    knownObjectNames = scene.get_known_object_names()
    knownObjectPoses = scene.get_object_poses(knownObjectNames)
    allSceneObjects = []
    
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
    # sub = rospy.Subscriber('/joint_states', JointState, pacer)
    
    # home the robot
    homeRobot()
    
    for name, pose in knownObjectPoses.items():
        allSceneObjects.append(parseObjData(name, pose))
        
    allSceneObjects.sort(key=lambda SceneObject: (SceneObject.position[0]))
    botIsIdle = True
    
        
    for obj in allSceneObjects:
        obj.pickPlacePipeline()
        scene.remove_world_object(obj.ID)
    


