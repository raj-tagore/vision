#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


def publish_pose():
    # Initialize ROS Node
    rospy.init_node('pose_publisher_node', anonymous=True)

    # Create a Publisher Object
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        # Create a PoseStamped message
        pose = PoseStamped()

        # Fill the header
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = ""

        # Fill the pose (position)
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0

        # Fill the pose (orientation) - Example: Quaternion for no rotation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # Publish the PoseStamped message
        pub.publish(pose)
        rospy.loginfo('PoseStamped message published')

        # Sleep for the remainder of the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass