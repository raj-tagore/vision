#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision.msg import ObjPose
import open3d as o3d
from sensor_msgs.msg import JointState

class Vision:
    def __init__(self):
        
        self.model = YOLO('/home/rajtagore/rgp2_ws/cv/model1.pt')
        
        self.obj_pose_pub = rospy.Publisher("/rgp2/obj_poses", ObjPose, queue_size=1)
        self.xyz_subscriber = rospy.Subscriber("/rgp2/xyz_uv", Float32MultiArray, self.get_xyz_uv)
        self.rgb_subscriber = rospy.Subscriber("camera/rgb/image_raw", Image, self.get_rgb)
        
        self.xyz = None
        self.rgb = None
        
    def get_xyz_uv(self, data):
        self.xyz = np.array(data.data).reshape(480, 460, 3)
        
    def get_rgb(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.rgb = cv_image
        
    def get_poses(self):
        
        if self.xyz is None or self.rgb is None:
            return
        
        xyz = self.xyz
        rgb = self.rgb
        
        results = self.model(rgb)
        detections = results[0].boxes
        
        for name, bb, conf in zip(detections.cls, detections.xyxy, detections.conf):
            name = str(name.cpu().numpy())
            bb = bb.cpu().numpy()
            conf = conf.cpu().numpy()
            x1, y1, x2, y2 = map(int, bb[:4])
            lb = 10
            ub = 450
            if lb<=x1<=ub and lb<=x2<=ub and lb<=y1<=ub and lb<=y2<=ub and conf>0.69:
                xyz_crop = xyz[y1:y2, x1:x2].reshape(-1, 3)
                xyz_crop = xyz_crop[~np.any(xyz_crop == 0, axis=1)]
                
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(xyz_crop)
                bbox = pcd.get_axis_aligned_bounding_box()
                center = bbox.get_center()  
                dims = bbox.get_max_bound() - bbox.get_min_bound()
                max_index = np.argmax(dims)
                if max_index == 0:
                    orientation = "x"
                elif max_index == 1:
                    orientation = "y"
                else:
                    orientation = "z"
                
                objpose = ObjPose()
                objpose.name = name
                objpose.center = center
                objpose.axis = orientation
                
                self.obj_pose_pub.publish(objpose)
                print("Obj: ", name, "@:", dims, "axis:", orientation, "published.")
                
def determineIfBotIsIdle(data):
    global botShouldScan
    magnitude = np.linalg.norm(data.velocity) 
    if magnitude < 0.02 and botShouldScan:
        rospy.sleep(6.9)
        vision.get_poses()
        botShouldScan = False
    elif magnitude >= 0.1:
        botShouldScan = True
    
if __name__ == "__main__":
    botShouldScan = True
    try: 
        rospy.init_node("vision_node")
        vision = Vision()
        vis = o3d.visualization.Visualizer()
        sub = rospy.Subscriber("/joint_states", JointState, determineIfBotIsIdle)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
        
        