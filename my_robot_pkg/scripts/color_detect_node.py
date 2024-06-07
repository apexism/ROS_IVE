#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class ColorDetectNode:
    def __init__(self):
        rospy.init_node('color_detect_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.color_pub = rospy.Publisher('color_detect', String, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        detected_color = self.detect_color(cv_image)
        if detected_color:
            self.color_pub.publish(detected_color)

    def detect_color(self, img):
        # HSV 변환 및 색상 검출 로직 (간소화됨)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 예를 들어 노란색 범위
        lower_yellow = np.array([22, 93, 0])
        upper_yellow = np.array([45, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        if np.any(mask):
            return "yellow"
        return None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ColorDetectNode()
    node.run()
