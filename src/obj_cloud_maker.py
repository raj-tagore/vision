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
import time


class Vision:
    def __init__(self):
        self.K = [554.3827128226441, 0.0, 320.5, 0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.K = np.array(self.K).reshape(3, 4)
        
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("panda_link0", "camera_depth_link", rospy.Time(0), rospy.Duration(4.0))

        self.model = YOLO('/home/rajtagore/rgp2_ws/cv/model1.pt')
        
        self.obj_clouds_pub = rospy.Publisher("/rgp2/obj_clouds", ObjCloud, queue_size=1)
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
        
    def bot_vision(self):
        
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
        self.tf_mtx[1][3] = pos[1]
        self.tf_mtx[2][3] = pos[2]
        
        # calculate xyz_uv
        for point in pc:
            xyz = np.array([point[0], point[1], point[2], 1]).T
            uv1 = np.dot(self.K, xyz) / xyz[2]
            xyz = np.dot(self.tf_mtx, xyz)
            self.xyz_uv[int(uv1[1]), int(uv1[0])] = [xyz[0], xyz[1], xyz[2]]
        
        # get bounding boxes from model
        results = self.model(rgb)
        detections = results[0].boxes
        
        for name, bb, conf in zip(detections.cls, detections.xyxy, detections.conf):
            
            name = str(name.cpu().numpy())
            bb = bb.cpu().numpy()
            conf = conf.cpu().numpy()
            x1, y1, x2, y2 = map(int, bb[:4])
            box = [x1, y1, x2, y2]
            
            if 10 <= x1 <= 480 and 10 <= x2 <= 480 and 10 <= y1 <= 480 and 10 <= y2 <= 480 and conf > 0.69:
                
                obj = Obj(name, box, conf)
                obj.pc = np.zeros((y2-y1, x2-x1, 3), dtype=np.float32)
                
                for x in range(x1, x2):
                    for y in range(y1, y2): 
                        obj.pc[y-y1, x-x1] = self.xyz_uv[y, x]
                        
                obj.publish(self.obj_clouds_pub)
                
                    


class Obj:
    def __init__(self, name, box, conf):
        self.name = name
        self.box = box
        self.conf = conf
        self.pc = []
        
        
    def publish(self, publisher):
        obj_cloud = ObjCloud()
        obj_cloud.name = self.name
        obj_cloud.size = list(self.pc.shape)
        obj_cloud.points = self.pc.flatten().tolist()
        
        publisher.publish(obj_cloud)
        
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('obj_cloud_maker', anonymous=True)
        vision = Vision()
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            vision.bot_vision()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass