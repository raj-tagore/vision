#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import sensor_msgs.point_cloud2 as pc2
import tf
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision.msg import ObjCloud


class Vision:
    def __init__(self):
        self.K = [554.3827128226441, 0.0, 320.5, 0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.K = np.array(self.K).reshape(3, 4)
        
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("panda_link0", "camera_depth_link", rospy.Time(0), rospy.Duration(4.0))
        
        self.xyz_uv_pub = rospy.Publisher("/rgp2/xyz_uv", Float32MultiArray, queue_size=1)
        self.pc_subscriber = rospy.Subscriber("/camera/depth/points", PointCloud2, self.get_pc)
        self.rgb_subscriber = rospy.Subscriber("camera/rgb/image_raw", Image, self.get_rgb)
        
        self.points = None
        self.rgbs = None
        self.tf_mtx = None
        self.xyz_uv = np.zeros((481, 641, 3), dtype=np.float32)
        
    def get_pc(self, data):
        self.points = np.array(list(pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)))
        # self.points = self.points[::10]
        
    def get_rgb(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.rgb = cv_image
        
    def get_xyz_uv(self):
        
        if self.points is None or self.rgb is None:
            return
        
        now = rospy.Time(0)
        pose = self.listener.lookupTransform("camera_depth_link", "panda_link0", now)
        rgb = self.rgb
        pc = self.points
        
        # calculate the transformation matrix
        pos = pose[0]
        ori = pose[1]
        quat = [ori[0], ori[1], ori[2], ori[3]]
        self.tf_mtx = tf.transformations.quaternion_matrix(quat)
        self.tf_mtx[0][3] = pos[0]
        
        pc[:, 3] = 1
         
        print(pc.shape)
        
            
            



if __name__ == '__main__':
    
    try:
        rospy.init_node('xyz_uv_maker', anonymous=True)
        vision = Vision()
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            vision.get_xyz_uv()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass