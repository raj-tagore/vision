#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import MoveAction, MoveGoal
from franka_gripper.msg import GraspAction, GraspGoal
import actionlib
import time

def move_gripper(width):
    """
    Function to open or close the gripper to a specific width.
    """
    move = MoveGoal()
    move.width = width  # Set the desired gripper width
    move.speed = 0.1  # Set the desired gripper speed
    client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
    client.wait_for_server()
    client.send_goal(move)
    client.wait_for_result()
    result = client.get_result()
    print(result)

def move_robot_to_pose(pose_publisher, x, y, z, ox=-3.14, oy=-0.01, oz=0.0):
    """
    Function to move the robot to a specific pose.
    """
    pose_msg = PoseStamped()
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation.x = ox
    pose_msg.pose.orientation.y = oy
    pose_msg.pose.orientation.z = oz

    pose_publisher.publish(pose_msg)

def grasp_object(width):
    """
    Function to grasp an object by closing the gripper to a specified width.
    """
    goal = GraspGoal()
    goal.width = width
    goal.epsilon.inner = 0.1
    goal.epsilon.outer = 0.1
    goal.force = 50
    goal.speed = 0.1
    client = actionlib.SimpleActionClient('franka_gripper/grasp', GraspAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration.from_sec(5))
    result = client.get_result()
    print(result)

def release_object_above_bin(pose_publisher, bin_position, release_width=0.08):
    """
    Function to move to the bin position and release the object.
    """
    # Move to position above the bin
    move_robot_to_pose(pose_publisher, bin_position[0], bin_position[1], bin_position[2] + 0.05)  # Adjust Z to be slightly above the bin
    rospy.sleep(2)  # Wait for the movement to complete

    # Open the gripper to release the object
    move_gripper(width=release_width)
    rospy.sleep(2)  # Ensure the object is released

def move_robot_and_handle_objects():
    rospy.init_node('robot_mover', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz

    pose_publisher = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)

    object_positions = [
        [0.363, -0.225, -0.081],
        [0.319, -0.022, -0.081],
        [0.173, 0.13, 0.019],
        [0.78, -0.16, -0.006],
        [0.533, 0.041, 0.025],
        [0.534, -0.33, 0.025]
    ]

    bin_position = [-0.35, -0.5, 0.5]

    for pos in object_positions:
        move_robot_to_pose(pose_publisher, pos[0], pos[1], pos[2] + 0.05)
        rospy.sleep(1)

        move_gripper(width=0.08)  # Open the gripper before picking
        rospy.sleep(1)

        move_robot_to_pose(pose_publisher, pos[0], pos[1], pos[2])
        rospy.sleep(1)

        grasp_width = 0.055  # Adjust this value based on the object size
        grasp_object(grasp_width)

        release_object_above_bin(pose_publisher, bin_position)

if __name__ == '__main__':
    try:
        move_robot_and_handle_objects()
    except rospy.ROSInterruptException:
        pass
