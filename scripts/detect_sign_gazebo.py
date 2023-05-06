#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
import torch

class SignDetection(object):
    def __init__(self):
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f'working on {device}')
        # yolo model for object detection
        # self.yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.yolo_model = torch.hub.load('/home/gbbyrd/.local/lib/python3.8/site-packages/yolov5', 'custom', path='/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/src/auefinals/src/scripts/yolov5s.pt', source='local')
        self.yolo_model.cuda()
        self.yolo_model.eval()
        
        rospy.init_node("sign_detector", anonymous=True)
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        self.count = 0
        
    def camera_callback(self, msg):
        try:
            self.cv2_image = cv2.resize(self.bridge_object.compressed_imgmsg_to_cv2(msg,desired_encoding="rgb8"), (400, 400), interpolation=cv2.INTER_AREA)
        except CvBridgeError as e:
            print(e) 
        self.count += 1
    
    def detect_stop_sign(self):
        # perform object detection on the camera image
        results = self.yolo_model(self.cv2_image)
        
        data = results.pandas().xyxy[0]

        # the stop sign is class 11. this determines if a stop sign was
        # detected in the camera frame
        is_stop = True if 11 in data['class'].values else False
        row = ...
        
        # if stop sign is detected, loop through to get the row of the class
        if is_stop:
            print("stop sign detected!")
            for i, row in enumerate(data.loc[:,'class']):
                if row == 11:
                    is_stop = True
                    row = data.iloc[i]
                    break
        
            # get the bounds of the box
            xmin = int(row.loc['xmin'])
            xmax = int(row.loc['xmax'])
            ymin = int(row.loc['ymin'])
            ymax = int(row.loc['ymax'])
            
            self.draw_box(xmin, xmax, ymin, ymax)
        
    def draw_box(self, xmin, xmax, ymin, ymax):
        # top
        cv2.line(self.cv2_image, (xmin, ymax), (xmax, ymax), (0,255,0) ,2)
        # bottom
        cv2.line(self.cv2_image, (xmin, ymin), (xmax, ymin), (0,255,0), 2)
        # left
        cv2.line(self.cv2_image, (xmin, ymin), (xmin, ymax), (0,255,0), 2)
        # right
        cv2.line(self.cv2_image, (xmax, ymin), (xmax, ymax), (0,255,0), 2)
            
    def run(self):
        rospy.sleep(3)
        while not rospy.is_shutdown():
            self.detect_stop_sign()
            cv2.imshow("Camera", cv2.cvtColor(self.cv2_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

def main():
    sign_detect = SignDetection()
    sign_detect.run()
    
if __name__=='__main__':
    main()
    
            
    