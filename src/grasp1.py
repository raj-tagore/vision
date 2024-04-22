#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import GripperTranslation
from geometry_msgs.msg import Vector3
from moveit_msgs.msg import Grasp
from moveit_commander import RobotCommander, MoveGroupCommander

grasp_pose = PoseStamped()
grasp_pose.header.frame_id = "base_frame"  # adjust accordingly to your TF tree
grasp_pose.pose.position.x = 0.25
grasp_pose.pose.position.y = 0.0
grasp_pose.pose.position.z = 0.3

# Assuming the orientation is along the z-axis (adjust if along x or y):
grasp_pose.pose.orientation.x = 0
grasp_pose.pose.orientation.y = 0
grasp_pose.pose.orientation.z = 0
grasp_pose.pose.orientation.w = 1

# Pre-grasp posture (e.g., fingers open)
pre_grasp_posture = JointTrajectory()
pre_grasp_posture.joint_names = ["gripper_joint"]
pre_grasp_point = JointTrajectoryPoint()
pre_grasp_point.positions = [0]  # open position
pre_grasp_point.time_from_start = rospy.Duration(2)
pre_grasp_posture.points.append(pre_grasp_point)

# Grasp posture (e.g., fingers closed)
grasp_posture = JointTrajectory()
grasp_posture.joint_names = ["gripper_joint"]
grasp_point = JointTrajectoryPoint()
grasp_point.positions = [1]  # closed position
grasp_point.time_from_start = rospy.Duration(2)
grasp_posture.points.append(grasp_point)

# Approach strategy
approach = GripperTranslation()
approach.direction.vector.z = 1  # Approach from above
approach.desired_distance = 0.1  # meters
approach.min_distance = 0.05

# Retreat strategy
retreat = GripperTranslation()
retreat.direction.vector.z = -1  # Retreat upwards
retreat.desired_distance = 0.1
retreat.min_distance = 0.05

grasp = Grasp()
grasp.id = "object1"
grasp.pre_grasp_posture = pre_grasp_posture
grasp.grasp_posture = grasp_posture
grasp.grasp_pose = grasp_pose
grasp.pre_grasp_approach = approach
grasp.post_grasp_retreat = retreat
grasp.grasp_quality = 1.0  # if applicable

grasps = [grasp]

# Initialize the ROS node
rospy.init_node('move_bot_node')

# Create a RobotCommander object
robot = RobotCommander()

# Create a MoveGroupCommander object for the arm
arm_group = MoveGroupCommander('panda_arm')

# Set the planning time
arm_group.set_planning_time(5)
arm_group.set_support_surface_name("table")
arm_group.pick("object1", grasps)

