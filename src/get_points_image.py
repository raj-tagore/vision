#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import struct

def callback(data):
    
    K = [554.3827128226441, 0.0, 320.5, 0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    K = np.array(K).reshape(3, 4)
    
    # Convert PointCloud2 to numpy array
    points = np.array(list(pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)))
    
    pixels = np.zeros((481, 641, 3), dtype=np.uint8)

    for point in points:

        xyz = np.array([point[0], point[1], point[2], 1]).T
        
        uv1 = np.dot(K, xyz)
        uv1 = uv1 / uv1[2]
        
        # The 'rgb' field is packed as a float, so we need to convert it to an integer
        # and then unpack it to get the RGB values.
        rgb = point[3]
        
        # Convert the float to a 32-bit binary representation
        binary_rgb = struct.unpack('I', struct.pack('f', rgb))[0]

        # Extract the RGB values
        r = (binary_rgb >> 16) & 0xFF
        g = (binary_rgb >> 8) & 0xFF
        b = binary_rgb & 0xFF
        
        pixels[int(uv1[1]), int(uv1[0])] = [b, g, r]
    
    resized = cv2.resize(pixels, (640, 480), interpolation=cv2.INTER_CUBIC)
    
    cropped = resized[:, :460]
    cv2.imshow("Depth Image", cropped)
    cv2.waitKey(3)
    

if __name__ == '__main__':
    try:
        rospy.init_node('pointcloud_to_image', anonymous=True)
        
        # Initialize CV Bridge
        bridge = CvBridge()
        
        # Create a subscriber to the point cloud data
        pc_subscriber = rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    cv2.destroyAllWindows()
