#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import numpy as np


class World:
    def __init__(self):
        self.image = None
        self.xyzC = None
        self.xyzW = []
        self.tfx = None
        inx = [554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0]
        self.inx = np.array(inx).reshape(3, 3)
        
    def get_tfx(self, msg):
        inv_tfx = np.array(msg.data).reshape(4, 4)
        self.tfx = np.linalg.inv(inv_tfx)
        
    def get_image(self, msg):
        self.image = msg
                
    def get_xyzC(self, msg):
        # Convert PointCloud2 to array
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            point_list.append(point)
        self.xyzC = np.array(point_list)
        
        self.get_xyzW()
        
    def get_xyzW(self):
        for point in self.xyzC[np.random.randint(1, 101)::100]:
            x = point[0]
            y = point[1]
            z = point[2]
            
            # Convert to homogeneous coordinates
            point = np.array([x, y, z, 1]).T
            # Apply the inverse transformation matrix
            point = np.dot(self.tfx, point)
            
            # Convert back to cartesian coordinates
            point = np.round(point[:3], 2)   
            if point.tolist() not in self.xyzW:
                self.xyzW.append(point.tolist())

        W_pub.publish(Float32MultiArray(data=np.array(self.xyzW).flatten().tolist()))
            
            
    
if __name__ == '__main__':
    
    world = World()
    
    rospy.init_node('rgp2_build_world')
    
    
    
    W_pub = rospy.Publisher('rgp2/world', Float32MultiArray, queue_size=10)
    T_sub = rospy.Subscriber('rgp2/camera_transform', Float32MultiArray, world.get_tfx)
    C_sub = rospy.Subscriber('camera/rgb/image_raw', Image, world.get_image)
    D_sub = rospy.Subscriber('camera/depth/points', PointCloud2, world.get_xyzC)
    
    rospy.spin()
    
        