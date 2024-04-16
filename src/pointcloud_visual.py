#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

def process_point_cloud(msg):
    # Convert PointCloud2 to array
    point_list = []
    for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
        point_list.append(point)
        
    points = np.array(point_list)
    pcd.points = o3d.utility.Vector3dVector(points)
    
    vis.update_geometry(pcd)
    
    vis.poll_events()
    vis.update_renderer()

if __name__ == '__main__':
    rospy.init_node('pointcloud_visualizer', anonymous=True)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=800, height=600)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], dtype=np.float64))
    vis.add_geometry(pcd)
    vis.get_view_control().set_front([1, 0, -1])  # Adjust camera front direction
    vis.get_view_control().set_lookat([0.69, 0, 0])  # Center the view on the point cloud
    vis.get_view_control().set_up([0, 0, -1])  # Adjust the up direction
    vis.get_view_control().set_zoom(0.069)  # Adjust the zoom level
    vis.poll_events()
    vis.update_renderer()
    rospy.Subscriber("/camera/depth/points", PointCloud2, process_point_cloud)
    rospy.spin()
