#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2

# thresholds1 = [np.array([0, 50, 50]), np.array([10,255,255])]
thresholds1 = [np.array([170, 200, 50]), np.array([180,255,255])]
thresholds2 = [np.array([0, 200, 50]), np.array([10,255,255])]

class ObjectDetection():
    def __init__(self):
        self.img   = None
        self.depth = None
        self.info  = None
        self.object_coord = np.zeros(3)
        self.bridge = CvBridge()
        self.pub_img   = rospy.Publisher('/object_detection/object_img', Image, queue_size=1)
        self.pub_depth = rospy.Publisher('/object_detection/object_depth', Image, queue_size=1)
        self.pub_info  = rospy.Publisher('/object_detection/camera_info', CameraInfo, queue_size=1)
        # self.sub_img   = rospy.Subscriber('/camera/rgb/image_raw', Image, self.update_img)
        # self.sub_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.update_depth)
        self.sub_img   = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.update_img)
        self.sub_depth = rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.update_depth)
        # self.sub_info  = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.update_info)
        self.sub_info  = rospy.Subscriber('/camera/depth_registered/camera_info', CameraInfo, self.update_info)
        # self.sub_point_clouds = rospy.Subscriber('/object_detection/points', PointCloud2, self.update_points)

    def update_img(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Process Image.
        self.object_detection()
        
    def update_depth(self, msg):
        # self.depth = self.bridge.imgmsg_to_cv2(msg) # for simulation
        depth = self.bridge.imgmsg_to_cv2(msg)/1000
        self.depth = depth.astype(np.float32) # for real-world
        # print(self.depth)

        # Process Image.
        self.object_detection()

    def update_info(self, msg):
        self.info = msg

    def update_points(self, msg):
        points = pc2.read_points(msg, skip_nans=True)
        points = np.asarray(list(points))
        print(points.shape)

        if points.shape[0] != 0:
            self.object_coord = np.mean(points, axis=0)

        print(self.object_coord)
            

    def object_detection(self):
        img_yuv = cv2.cvtColor(self.img, cv2.COLOR_BGR2YUV) # RGB => YUV(YCbCr)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        img_yuv[:,:,0] = clahe.apply(img_yuv[:,:,0]) # 輝度にのみヒストグラム平坦化
        img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR) # YUV --> RGB
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # BGR --> HSV

        hsv = cv2.GaussianBlur(hsv, (5,5), 3) # 画像平滑化
        
        bin1 = cv2.inRange(hsv, thresholds1[0], thresholds1[1])
        bin2 = cv2.inRange(hsv, thresholds2[0], thresholds2[1])
        mask = bin1 + bin2
        # mask = bin1
        # print(self.depth)
        masked_depth = self.depth.copy()
        masked_depth[mask == 0] = np.nan
        # depth = masked_depth[~np.isnan(masked_depth)]
        # print(np.mean(depth))
        masked_img = cv2.bitwise_and(img, img, mask= mask) # 色抽出

        now = rospy.Time.now()

        msg_img = self.bridge.cv2_to_imgmsg(masked_img, "bgr8")
        msg_img.header.stamp = now
        msg_img.header.frame_id = "camera_rgb_optical_frame"

        msg_depth = self.bridge.cv2_to_imgmsg(masked_depth)
        msg_depth.header.stamp = now
        msg_depth.header.frame_id = "camera_depth_optical_frame"

        self.pub_img.publish(msg_img)
        self.pub_depth.publish(msg_depth)

        info = self.info
        info.header.stamp = now
        self.pub_info.publish(info)
    

def main():
    rospy.loginfo("Start to detect object.")
    rospy.init_node("object_detection")
    tucker = ObjectDetection()
    rospy.spin()

if __name__ == "__main__":
    main()
