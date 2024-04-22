#! /usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, TwistStamped
import sys
import tf
from moveit.task_constructor import core, stages
from moveit_commander.roscpp_initializer import roscpp_initialize
import time
from moveit_commander.roscpp_initializer import roscpp_initialize


if __name__ == '__main__':

    # sampleObjPose = PoseStamped()
    # sampleObjPose.header.frame_id = "panda_link0"
    # sampleObjPose.pose.position.x = 0.070505
    # sampleObjPose.pose.position.y = 0.12949
    # sampleObjPose.pose.position.z = 0.065792
    # axis = 'y'
    # orientationMatrix = [[0, -1, 0, 0], 
    #                      [1, 0, 0, 0], 
    #                      [0, 0, 1, 0], 
    #                      [0, 0, 0, 1]]
    # quaternion = tf.transformations.quaternion_from_matrix(orientationMatrix)
    # sampleObjPose.pose.orientation.x = quaternion[0]
    # sampleObjPose.pose.orientation.y = quaternion[1]
    # sampleObjPose.pose.orientation.z = quaternion[2]
    # sampleObjPose.pose.orientation.w = quaternion[3]
    # sampleObjRadius = 0.05
    
    roscpp_initialize("pickplace")
    
    rospy.init_node('pick', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    
    knownObjectNames = scene.get_known_object_names()
    knownObjectPoses = scene.get_object_poses(knownObjectNames)
    
    task = core.Task("PickPlace1")
    task.enableIntrospection()
    task.add(stages.CurrentState("current_state"))
    
    pipeline = core.PipelinePlanner()
    pipeline.planner = "RRTConnectkConfigDefault"
    planners = [("panda_manipulator", pipeline)]
    
    task.add(stages.Connect("connect1", planners))
    
    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = 0.2
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "close"
    grasp_generator.setMonitoredStage(task["current_state"])
    
    simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")
    simpleGrasp.setIKFrame("panda_link8")
    
    pick = stages.Pick(simpleGrasp, "Pick")
    pick.eef = "panda_link8"
    pick.object = knownObjectNames[0]
    
    # Twist to approach
    approach = TwistStamped()
    approach.header.frame_id = "world"
    approach.twist.linear.z = -1.0
    pick.setApproachMotion(approach, 0.03, 0.1)
    
    # Twist to lift
    lift = TwistStamped()
    lift.header.frame_id = "panda_hand"
    lift.twist.linear.z = -1.0
    pick.setLiftMotion(lift, 0.03, 0.1)
    
    task.add(pick)
    
    if task.plan():
        task.publish(task.solutions[0])
    
    time.sleep(3600)
    