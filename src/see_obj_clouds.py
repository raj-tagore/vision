#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from vision.msg import ObjCloud
import open3d as o3d

def create_plots():
    
    global obj_list
    
    # Placeholder code
    for i in range(len(obj_list)):
        name, points = obj_list[i]
        points = points.reshape(-1, 3)
        
        points = points[~np.any(points == 0, axis=1)]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Compute smallest bounding box around pointcloud
        bbox = pcd.get_axis_aligned_bounding_box()
        
        # Get the center of the bounding box
        center = bbox.get_center()

        # Print the center coordinates
        print("Center coordinates:", center)

        # Draw the bounding box along with pointclouds
        o3d.visualization.draw_geometries([pcd, bbox], window_name=name, width=800, height=600)
        
        
    
def process_point_cloud(msg):
    
    global obj_list
    name = msg.name
    size = msg.size
    
    # print(np.array(msg.points).reshape(size).shape)
    
    obj_list.append([name, np.array(msg.points).reshape(size)])
    
    
    if len(obj_list) >= 10:
        
        create_plots()
        rospy.signal_shutdown("All objects have been visualized once.")   
    
    
    
    
if __name__ == '__main__':
    rospy.init_node('see_point_clouds', anonymous=True)
    
    obj_list = []
    
    rospy.Subscriber("/rgp2/obj_clouds", ObjCloud, process_point_cloud)
    rospy.spin()