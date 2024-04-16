#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import open3d as o3d
import numpy as np

def process_world(msg):
    
    points = np.array(msg.data).reshape(-1, 3)
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # # Generate random colors for each point
    # colors = np.random.uniform(0, 1, size=(points.shape[0], 3))
    # # Assign colors to the point cloud
    # pcd.colors = o3d.utility.Vector3dVector(colors)
    
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

if __name__ == '__main__':
    rospy.init_node('world_visualizer', anonymous=True)

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=800, height=600)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], dtype=np.float64))
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    vis.add_geometry(coordinate_frame)
    vis.add_geometry(pcd)
    vis.get_view_control().set_front([-3, 0, 1])  # Adjust camera front direction
    vis.get_view_control().set_lookat([0.69, 0, 0])  # Center the view on the point cloud
    vis.get_view_control().set_up([0, 0, 1])  # Adjust the up direction
    vis.get_view_control().set_zoom(0.69)  # Adjust the zoom level
    vis.poll_events()
    vis.update_renderer()
    rospy.Subscriber("/rgp2/world", Float32MultiArray, process_world)
    rospy.spin()
