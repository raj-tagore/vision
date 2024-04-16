#! /usr/bin/env python

import rospy
from vision.msg import ObjPose
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys
import tf
import numpy as np

def addObjectToMoveitScene(data):
    name = data.name
    center = data.center
    axis = data.axis
    
    # check if object needs to be added or not
    knownObjectNames = scene.get_known_object_names()
    knownObjectPoses = scene.get_object_poses(knownObjectNames)
    
    # x:=-0.10 y:=0 z:=0.615 is the robot-base's position
    for _, pose in knownObjectPoses.items():
        center_diff = np.array([(pose.position.x + 0.10) - center[0],
                    pose.position.y - center[1],
                    (pose.position.z - 0.615) - center[2]])
        dist = np.linalg.norm(center_diff)
        if dist < 0.05:
            print("Object already exists in the scene, Not adding it again.")
            return
        
    pose = PoseStamped()
    pose.header.frame_id = "panda_link0"
    pose.pose.position.x = center[0]
    pose.pose.position.y = center[1]
    pose.pose.position.z = center[2]
    
    if axis == "x":
        orientationMatrix = [[1, 0, 0, 0], 
                             [0, 1, 0, 0], 
                             [0, 0, 1, 0], 
                             [0, 0, 0, 1]]
    elif axis == "y":
        orientationMatrix = [[0, -1, 0, 0], 
                             [1, 0, 0, 0], 
                             [0, 0, 1, 0], 
                             [0, 0, 0, 1]]
    elif axis == "z":
        orientationMatrix = [[0, 0, -1, 0], 
                             [0, 1, 0, 0], 
                             [1, 0, 0, 0], 
                             [0, 0, 0, 1]]
        
    quaternion = tf.transformations.quaternion_from_matrix(orientationMatrix)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    
    global uniqueId
    uniqueName = str(uniqueId)+"_"+name+"_"+axis
    uniqueId += 1
    
    
    if name == "0.0":
        size = [0.2, 0.069, 0.069]
    elif name == "1.0":
        size = [0.12, 0.069, 0.069]
    else:
        size = [0.03, 0.03, 0.02]
    
    scene.add_box(uniqueName, pose, size)
    

if __name__ == "__main__":
    
    try:
        uniqueId = 0
        
        rospy.init_node("world_maker_node")
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        scene.clear()
        
        sub = rospy.Subscriber("/rgp2/obj_poses", ObjPose, addObjectToMoveitScene)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass