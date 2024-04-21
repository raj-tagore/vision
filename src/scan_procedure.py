#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import tf
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import open3d as o3d
import moveit_commander
import sys
import cv2

def get_xyz_uv():
    listener = tf.TransformListener()
    listener.waitForTransform("panda_link0", "camera_depth_link", rospy.Time(0), rospy.Duration(4.0))
    
    K = [554.3827128226441, 0.0, 320.5, 0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    K = np.array(K).reshape(3, 4)
    pose = listener.lookupTransform("panda_link0", "camera_depth_link", rospy.Time(0))
    pos = pose[0]
    ori = pose[1]
    quat = [ori[0], ori[1], ori[2], ori[3]]
    tf_mtx = tf.transformations.quaternion_matrix(quat)
    tf_mtx[0][3] = pos[0]
    tf_mtx[1][3] = pos[1]
    tf_mtx[2][3] = pos[2]
    
    data = rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=1.0)
    
    points = np.array(list(pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)))
    xyz_uv = np.zeros((480, 460, 3), dtype=np.float32)
    
    for point in points:
        xyz = np.array([point[0], point[1], point[2], 1]).T
        uv1 = np.dot(K, xyz) / xyz[2]
        if uv1[0] < 0 or uv1[0] >= 460 or uv1[1] < 0 or uv1[1] >= 480:
            continue
        xyz = np.dot(tf_mtx, xyz)
        xyz_uv[int(uv1[1]), int(uv1[0])] = [xyz[0], xyz[1], xyz[2]]
    
    print("\n ------- \n\n xyz_uv has been calculated")
    return xyz_uv

def build_world(xyz_uv):
    model = YOLO('/home/rajtagore/rgp2_ws/cv/model1.pt')
    
    rgb = rospy.wait_for_message("camera/rgb/image_raw", Image, timeout=1.0)
    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb, 'bgr8')
    
    results = model(rgb)
    detections = results[0].boxes
    
    for name, bb, conf in zip(detections.cls, detections.xyxy, detections.conf):
        name = str(name.cpu().numpy())
        bb = bb.cpu().numpy()
        conf = conf.cpu().numpy()
        x1, y1, x2, y2 = map(int, bb[:4])
        lb = 1
        ub = 455
        spawn = True
        if lb<=x1<=ub and lb<=x2<=ub and lb<=y1<=ub and lb<=y2<=ub and conf>0.69:
            xyz_crop = xyz_uv[y1:y2, x1:x2].reshape(-1, 3)
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
            
            knownObjectNames = scene.get_known_object_names()
            knownObjectPoses = scene.get_object_poses(knownObjectNames)
            
            for _, pose in knownObjectPoses.items():
                center_diff = np.array([(pose.position.x + 0.10) - center[0],
                            pose.position.y - center[1],
                            (pose.position.z - 0.615) - center[2]])
                dist = np.linalg.norm(center_diff)
                if dist < 0.03:
                    print("Object already exists in the scene, Not adding it again.")
                    spawn = False
                    break
                
            if spawn:    
                objPose = PoseStamped()
                objPose.header.frame_id = "panda_link0"
                objPose.pose.position.x = center[0]
                objPose.pose.position.y = center[1]
                objPose.pose.position.z = center[2]
                
                if orientation == "x":
                    objPose.pose.orientation.x = 0.0
                    objPose.pose.orientation.y = 0.0
                    objPose.pose.orientation.z = 0.0
                    objPose.pose.orientation.w = 1.0
                elif orientation == "y":
                    objPose.pose.orientation.x = 0.0
                    objPose.pose.orientation.y = 0.0
                    objPose.pose.orientation.z = 0.707
                    objPose.pose.orientation.w = 0.707
                else:
                    objPose.pose.orientation.x = 0.0
                    objPose.pose.orientation.y = -0.707
                    objPose.pose.orientation.z = 0.0
                    objPose.pose.orientation.w = 0.707
                
                global uniqueId
                uniqueName = str(uniqueId)+"_"+name+"_"+orientation
                uniqueId += 1
                
                if name == "0.0":
                    size = [0.2, 0.069, 0.069]
                elif name == "1.0":
                    size = [0.12, 0.069, 0.069]
                else:
                    size = [0.03, 0.03, 0.02]
                
                print("Adding object to the scene")
                scene.add_box(uniqueName, objPose, size)
                    
            
    

def makePose(coordinates):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "panda_link0" 
    
    pose.pose.position.x = coordinates[0]
    pose.pose.position.y = coordinates[1]
    pose.pose.position.z = 0.5
    
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    
    return pose

def publishPose(pose):
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    time = rospy.Time.now()
    while not rospy.is_shutdown():
        pub.publish(pose)
        if rospy.Time.now() - time > rospy.Duration(6.9):
            break
        rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('move_bot', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()
    
    uniqueId = 0
    
    coordinatesList = [[-0.05, -0.45], [0.25, 0.00], [-0.05, 0.40], 
                       [0.30, -0.40], [0.50, 0.00], [0.30, 0.35]]
    
    try:
        for coordinate in coordinatesList:
            pose = makePose(coordinate)
            publishPose(pose)
            xyz_uv = get_xyz_uv()
            build_world(xyz_uv)
            
    except rospy.ROSInterruptException:
        pass
