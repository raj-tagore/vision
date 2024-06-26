#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

model = YOLO('/home/rajtagore/rgp2_ws/cv/model1.pt')

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv_image = cv_image[:, :460]
    results = model(cv_image)
    detections = results[0].boxes
    for name, bb, conf in zip(detections.cls, detections.xyxy, detections.conf):
        name = name.cpu().numpy()
        bb = bb.cpu().numpy()
        conf = conf.cpu().numpy()
        x1, y1, x2, y2 = map(int, bb[:4])
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label_text = f"{name}: {conf:.2f}"
        cv2.putText(cv_image, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
    cv2.imshow("Video Feed", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('video_subscriber_node')
    rospy.Subscriber("camera/rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()