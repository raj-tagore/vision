#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from vision.msg import ObjCloud
import open3d as o3d
from geometry_msgs.msg import PoseStamped
import tf
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal
from franka_gripper.msg import MoveAction, MoveGoal
import time

def process_objects(msg):
    
    global obj_list
    name = msg.name
    points = np.array(msg.points).reshape(-1, 3)
    
    object1 = Object(name, points)
    for o in obj_list:
        if o.name == object1.name:
            if np.linalg.norm(o.center - object1.center) < 0.05:
                return 
    object1.id = obj_list.__len__()
    obj_list.append(object1)
    
    pick_place()
      
class Object:
    def __init__(self, name, points):
        self.id = 0
        self.name = name
        self.points = points
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.bbox = self.pcd.get_axis_aligned_bounding_box()
        self.center = self.bbox.get_center()
        self.dimensions = self.bbox.get_max_bound() - self.bbox.get_min_bound()
        
        max_index = np.argmax(self.dimensions)
        if max_index == 0:
            self.orientation = "x"
        elif max_index == 1:
            self.orientation = "y"
        else:
            self.orientation = "z"
            
def pick_place():
    
    global obj_list
    global picked_objs
    
    for o in obj_list:
        if o.id not in picked_objs:
            picked_objs.append(o.id)
            print("Picking up object:", o.name, "with center: ", o.center, "and orientation:", o.orientation)
            
            open_gripper()
            rospy.sleep(5)
            pre_pickup(o.center, o.orientation)
            grasp(o.name)
            rospy.sleep(5)
            post_pickup(o.name)
            open_gripper()
            rospy.sleep(5)
            move_to_default()
            print("pickup task completed")
            
            
def pre_pickup(center, orientation):
    move_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)

    # set the pose to move to before gripping the object
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = ""
    if orientation == "x":
        pose.pose.position.x = center[0] 
        pose.pose.position.y = center[1]
        pose.pose.position.z = center[2] 
        
        rot_matrix = np.array([[1, 0, 0, 0], 
                               [0, -1, 0, 0], 
                               [0, 0, -1, 0], 
                               [0, 0, 0, 1]])
        quaternion = tf.transformations.quaternion_from_matrix(rot_matrix)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
     
    elif orientation == "y":
        pose.pose.position.x = center[0] 
        pose.pose.position.y = center[1]
        pose.pose.position.z = center[2] 
        
        rot_matrix = np.array([[0, -1, 0, 0], 
                               [-1, 0, 0, 0], 
                               [0, 0, -1, 0], 
                               [0, 0, 0, 1]])
        quaternion = tf.transformations.quaternion_from_matrix(rot_matrix)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
    else:
        pose.pose.position.x = center[0] 
        pose.pose.position.y = center[1]
        pose.pose.position.z = center[2]
        
        rot_matrix = np.array([[0, 0, 1, 0], 
                               [0, 1, 0, 0], 
                               [1, 0, 0, 0], 
                               [0, 0, 0, 1]])
        quaternion = tf.transformations.quaternion_from_matrix(rot_matrix)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
    now = time.time()
    while time.time() - now < 15:
        move_pub.publish(pose)
        rospy.sleep(0.1)
    print("Moved to pre-pickup position")
    
def post_pickup(name):
    move_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
    
    if name == 0:
        pose = PoseStamped()
        pose.pose.position.x = -0.35
        pose.pose.position.y = -0.5
        pose.pose.position.z = 0.5
        rot_matrix = np.array([[1, 0, 0, 0], 
                            [0, -1, 0, 0], 
                            [0, 0, -1, 0], 
                            [0, 0, 0, 1]])
        
        orientation = tf.transformations.quaternion_from_matrix(rot_matrix)
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
    else:
        pose = PoseStamped()
        pose.pose.position.x = -0.35
        pose.pose.position.y = 0.5
        pose.pose.position.z = 0.5
        rot_matrix = np.array([[1, 0, 0, 0], 
                            [0, -1, 0, 0], 
                            [0, 0, -1, 0], 
                            [0, 0, 0, 1]])
        
        orientation = tf.transformations.quaternion_from_matrix(rot_matrix)
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
    
    now = time.time()
    while time.time() - now < 15:
        move_pub.publish(pose)
        rospy.sleep(0.1)
    print("Moved to post-pickup position")
    
def move_to_default():
    move_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    pose = PoseStamped()
    pose.pose.position.x = 0.15
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.5
    rot_matrix = np.array([[1, 0, 0, 0], 
                           [0, -1, 0, 0], 
                           [0, 0, -1, 0], 
                           [0, 0, 0, 1]])
    
    orientation = tf.transformations.quaternion_from_matrix(rot_matrix)
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]
    
    now = time.time()
    while time.time() - now < 5:
        move_pub.publish(pose)
        rospy.sleep(0.1)
    print("Moved to default position")
    
def open_gripper():
    move = MoveGoal()
    move.width = 0.08
    move.speed = 0.1
    client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
    client.wait_for_server()
    client.send_goal(move)
    client.wait_for_result()
    result = client.get_result()
    print("Opened gripper")
    print(result)
    
def grasp(name):
    goal = GraspGoal()
    if name == 0 or 1:
        goal.width = 0.055
    else :
        goal.width = 0.03
    goal.epsilon.inner = 0.1
    goal.epsilon.outer = 0.1
    goal.force = 50
    goal.speed = 0.1
    client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration.from_sec(5))
    result = client.get_result()
    print("Grasped object")
    print(result)

if __name__ == "__main__":
    rospy.init_node('get_obj_poses', anonymous=True)
    
    move_to_default()

    obj_list = []
    picked_objs = []
    
    sub = rospy.Subscriber("/rgp2/obj_clouds", ObjCloud, process_objects)
    rospy.spin()
    
    